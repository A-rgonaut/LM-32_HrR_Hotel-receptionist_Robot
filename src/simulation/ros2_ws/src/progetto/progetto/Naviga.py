import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped

from math import sqrt, atan2, cos, sin, pi
import array

from progetto.Pianifica import astar_8conn

class Naviga:
    def __init__(self, nodo_padre):
        self.nodo = nodo_padre  # Riferimento all'Arbitraggio (che è il nodo ROS)

        # --- Configurazione QoS per la Mappa ---
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # --- Variabili Navigazione ---
        self.map_data = None
        self.map_info = None
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.robot_pose = None  # (x, y)
        self.robot_theta = 0.0  # yaw in radianti

        self.current_path_world = [] # Lista di waypoint (x, y)
        self.current_goal_idx = 0

        # --- Parametri Controllo ---
        self.Kp_linear = 0.5
        self.Kp_angular = 2.0
        self.dist_threshold = 0.15 # 15 cm di tolleranza

        self.pub_cmd_vel = self.nodo.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.nodo.create_publisher(Path, '/pianifica_path', 10)

        self.nodo.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.nodo.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.nodo.get_logger().info("Naviga...")

        self.timer = self.nodo.create_timer(0.1, self.control_loop)

    # --- 1. LOGICA DI CONTROLLO (Chiamata dall'Arbitraggio) ---
    def control_loop(self):
        # Se non ho la posa o la mappa, non posso fare nulla
        if self.robot_pose is None or self.map_data is None:
            return
        # Se ho una destinazione ma non ho ancora un percorso, PIANIFICO
        if not self.current_path_world and self.nodo.destinazione_target:
            self.pianifica()
            return
        if not self.current_path_world:
            return
        # Se ho finito il percorso
        if self.current_goal_idx >= len(self.current_path_world):
            self.stop_robot()
            self.nodo.get_logger().info("Naviga: Destinazione raggiunta!")
            # Segnalo all'arbitraggio che ho finito
            self.nodo.raggiunta_destinazione = True
            # Pulisco il path per la prossima volta
            self.current_path_world = []
            return

        # --- INSEGUIMENTO WAYPOINT ---
        tx, ty = self.current_path_world[self.current_goal_idx]
        rx, ry = self.robot_pose

        dx = tx - rx
        dy = ty - ry
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)

        # Errore angolare normalizzato (-PI a +PI)
        angle_error = angle_to_goal - self.robot_theta
        while angle_error > pi:
            angle_error -= 2 * pi
        while angle_error < -pi:
            angle_error += 2 * pi

        cmd = Twist()

        # Se sono vicino al waypoint, passo al prossimo
        if distance < self.dist_threshold:
            self.current_goal_idx += 1
        else:
            # Sterzata
            cmd.angular.z = self.Kp_angular * angle_error

            # Velocità lineare: vado veloce solo se sono allineato
            if abs(angle_error) < 0.5:  # ~30 gradi
                cmd.linear.x = min(self.Kp_linear * distance, 0.4)  # Max 0.4 m/s
            # elif abs(angle_error) < 1.0:
                # cmd.linear.x = 0.05  # Avanza piano mentre giri
            else:
                cmd.linear.x = 0.0  # Giro sul posto se disallineato

            self.pub_cmd_vel.publish(cmd)

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())

    # --- 2. PIANIFICAZIONE (A*) ---
    def pianifica(self):
        self.nodo.raggiunta_destinazione = False
        start_pose = self.robot_pose
        goal_pose = self.nodo.destinazione_target

        if not start_pose or not goal_pose:
            return

        self.nodo.get_logger().info(f"Naviga: Calcolo percorso da {start_pose} a {goal_pose}...")

        # Coordinate Griglia
        sx, sy = self.world_to_grid(start_pose[0], start_pose[1])
        gx, gy = self.world_to_grid(goal_pose[0], goal_pose[1])

        w = self.map_info.width
        h = self.map_info.height

        # Bounding Box per ottimizzare A*
        dx = abs(sx - gx)
        dy = abs(sy - gy)
        margin = max(30, int(0.5 * max(dx, dy)))
        minx = max(0, min(sx, gx) - margin)
        maxx = min(w - 1, max(sx, gx) + margin)
        miny = max(0, min(sy, gy) - margin)
        maxy = min(h - 1, max(sy, gy) + margin)
        bbox = (minx, maxx, miny, maxy)

        # Chiamata A* (dal file Pianifica.py)
        path_grid = astar_8conn(self.map_data, w, h, (sx, sy), (gx, gy), bbox=bbox, allow_unknown=False)

        if not path_grid:
            self.nodo.get_logger().warning("A* nel BBox fallito, provo mappa intera...")
            path_grid = astar_8conn(self.map_data, w, h, (sx, sy), (gx, gy), bbox=None, allow_unknown=False)

        if path_grid:
            self.nodo.get_logger().info(f"Path trovato: {len(path_grid)} passi.")

            # Semplificazione e conversione
            path_semplificato = self.simplify_path(path_grid)
            self.current_path_world = []

            # Creazione messaggio Path per RViz
            ros_path = Path()
            ros_path.header.frame_id = "map"
            ros_path.header.stamp = self.nodo.get_clock().now().to_msg()

            for cell in path_semplificato:
                wx, wy = self.grid_to_world(cell[0], cell[1])
                self.current_path_world.append((wx, wy))

                # Aggiungo a messaggio RViz
                ps = PoseStamped()
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ros_path.poses.append(ps)

            self.pub_path.publish(ros_path)
            self.current_goal_idx = 0
        else:
            self.nodo.get_logger().error("Impossibile trovare un percorso!")
            # Per evitare loop infiniti, diciamo che siamo arrivati (fallimento)
            self.nodo.raggiunta_destinazione = True

    # --- 3. GESTIONE MAPPA (Gonfiaggio) ---
    def map_callback(self, msg):
        if self.map_data is None:
            self.nodo.get_logger().info(f"Mappa ricevuta: {msg.info.width}x{msg.info.height}")

        self.map_info = msg.info
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        SAFETY_RADIUS = 0.30 # Metri di sicurezza attorno agli ostacoli
        self.map_data = self.inflate_map(msg.data, msg.info.width, msg.info.height, SAFETY_RADIUS)

    def inflate_map(self, data, width, height, robot_radius_meters):
        cells_radius = int(robot_radius_meters / self.resolution)
        if cells_radius <= 0:
            return list(data)

        new_map = list(data) # Copia mutabile
        # Scansioniamo per ostacoli (valore 100)
        for y in range(height):
            for x in range(width):
                if data[y * width + x] == 100:
                    # Espandi
                    for dy in range(-cells_radius, cells_radius + 1):
                        for dx in range(-cells_radius, cells_radius + 1):
                            if dx*dx + dy*dy <= cells_radius*cells_radius:
                                nx, ny = x + dx, y + dy
                                if 0 <= nx < width and 0 <= ny < height:
                                    new_map[ny * width + nx] = 100
        return new_map

    # --- 4. GESTIONE ODOMETRIA ---
    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_theta = atan2(siny_cosp, cosy_cosp)

    # --- 5. UTILS ---
    def simplify_path(self, path_grid):
        """Rimuove i punti intermedi se la direzione non cambia"""
        if len(path_grid) < 3:
            return path_grid
        new_path = [path_grid[0]]

        # Calcoliamo delta iniziale
        last_dx = path_grid[1][0] - path_grid[0][0]
        last_dy = path_grid[1][1] - path_grid[0][1]

        for i in range(2, len(path_grid)):
            curr_x, curr_y = path_grid[i]
            prev_x, prev_y = path_grid[i-1]

            curr_dx = curr_x - prev_x
            curr_dy = curr_y - prev_y

            # Se cambia direzione, salvo il punto di svolta
            if curr_dx != last_dx or curr_dy != last_dy:
                new_path.append(path_grid[i-1])
                last_dx = curr_dx
                last_dy = curr_dy

        new_path.append(path_grid[-1]) # Aggiungo sempre il goal
        return new_path

    def world_to_grid(self, wx, wy):
        if self.resolution == 0:
            return 0, 0
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = (gx * self.resolution) + self.origin_x
        wy = (gy * self.resolution) + self.origin_y
        return wx, wy

    def reset(self):
        pass
        # self.current_path_world = []
        # self.current_goal_idx = 0
        # self.stop_robot()

    def esegui(self, testo):
        pass
