import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from math import sqrt

from progetto.utils import Luogo
"""
class Agent():
    def __init__(self, start, goal, mappa):
        self.start = start
        self.goal  = goal
        self.mappa = mappa

    def next_states(self, state):
        raise NotImplementedError("Implementare next_states nella sottoclasse!")

class AgentBFS(Agent):
    def __init__(self, start, goal, mappa):
        super().__init__(start, goal, mappa)
        self.frontier = [[start]]

    def sort_frontier(self):
        pass

    def bfs(self):
        while self.frontier:
            path = self.frontier.pop(0)
            state = path[-1]
            if state == self.goal:
                return path
            for next_state in self.next_states(state):
                if next_state not in path:
                    self.frontier.append(path + [next_state])
            self.sort_frontier()

class AgentGreedy(AgentBFS):
    def __init__(self, start, goal, mappa):
        super().__init__(start, goal, mappa)

    def heuristic(self, path):
        return sqrt((path[-1].x - self.goal.x) ** 2 +
                    (path[-1].y - self.goal.y) ** 2)

    def sort_frontier(self):
        self.frontier.sort(key=lambda path: self.heuristic(path))

class AgentAStar(AgentGreedy):
    def __init__(self, start, goal, mappa):
        super().__init__(start, goal, mappa)

    def heuristic(self, path):
        return super().heuristic(path) + len(path)

class AgentAStarGrid(AgentAStar):
    def __init__(self, start, goal, mappa, width, height):
        super().__init__(start, goal, mappa)
        self.width = width
        self.height = height
        self.moves = [[1, 0], [-1, 0], [0, 1], [0, -1]]

    def next_states(self, state):
        vicini = []
        x, y = state.x, state.y
        for dx, dy in self.moves:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                index = ny * self.width + nx
                cella = self.mappa[index]
                if cella == 0:
                    vicini.append(Luogo(-1, nx, ny))
        return vicini
"""
import heapq
from math import sqrt

SQRT2 = sqrt(2.0)

def astar_8conn(map_data, width, height, start, goal, bbox=None, allow_unknown=False):
    """
    start, goal: (gx, gy)
    bbox: (minx, maxx, miny, maxy) oppure None
    allow_unknown: se True tratta -1 come libero
    ritorna: lista di (x,y) oppure None
    """

    def in_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    def in_bbox(x, y):
        if bbox is None:
            return True
        minx, maxx, miny, maxy = bbox
        return (minx <= x <= maxx) and (miny <= y <= maxy)

    def is_free(x, y):
        v = map_data[y * width + x]  # -1 unknown, 0 free, 100 occupied (tipico)
        if v == 0:
            return True
        if allow_unknown and v == -1:
            return True
        return False

    # euristica octile (adatta a 8-conn con costi 1 e sqrt(2))
    def h(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

    if not in_bounds(*start) or not in_bounds(*goal):
        return None
    if not in_bbox(*start) or not in_bbox(*goal):
        return None
    if not is_free(*start) or not is_free(*goal):
        return None

    moves = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, SQRT2), (1, -1, SQRT2), (-1,  1, SQRT2), (-1, -1, SQRT2)
    ]

    open_heap = []
    heapq.heappush(open_heap, (h(start, goal), 0.0, start))

    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        f, g, cur = heapq.heappop(open_heap)

        if cur in closed:
            continue
        if cur == goal:
            # reconstruct
            path = [cur]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        closed.add(cur)
        cx, cy = cur

        for dx, dy, cost in moves:
            nx, ny = cx + dx, cy + dy

            if not in_bounds(nx, ny) or not in_bbox(nx, ny):
                continue
            if not is_free(nx, ny):
                continue

            # evita "taglio dell'angolo" sui diagonali:
            if dx != 0 and dy != 0:
                if not (is_free(cx + dx, cy) and is_free(cx, cy + dy)):
                    continue

            nb = (nx, ny)
            tentative_g = g + cost

            if tentative_g < g_score.get(nb, float("inf")):
                came_from[nb] = cur
                g_score[nb] = tentative_g
                heapq.heappush(open_heap, (tentative_g + h(nb, goal), tentative_g, nb))

    return None

class Pianifica(Node):
    def __init__(self):
        super().__init__("Pianifica")

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_data = None
        self.map_info = None
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.robot_pose = None  # (metri)
        self.goal_pose  = None  # (metri)

        self.path_pub = self.create_publisher(
            Path,
            '/pianifica_path',
            10
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos  # 10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.get_logger().info("Pianificatore in attesa della mappa...")

    def inflate_map(self, data, width, height, robot_radius_meters):
        """
        Ingrassa gli ostacoli di un raggio pari a robot_radius_meters.
        """
        import array

        # Calcola quante celle servono (es: 0.20m / 0.05m/pixel = 4 celle)
        cells_radius = int(robot_radius_meters / self.resolution)
        if cells_radius <= 0:
            return data

        # Convertiamo la tupla immutabile in una lista modificabile (o array per velocità)
        # Usiamo 0 per libero, 100 per occupato
        new_map = list(data)

        length = len(data)

        # Scorriamo la mappa originale per trovare gli ostacoli
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                # Se troviamo un muro (o ignoto se vuoi essere prudente)
                if data[idx] == 100:
                    # Espandiamo il muro ai vicini nel raggio
                    for dy in range(-cells_radius, cells_radius + 1):
                        for dx in range(-cells_radius, cells_radius + 1):
                            # Controllo cerchio (distanza euclidea) per non fare muri quadrati
                            if dx*dx + dy*dy <= cells_radius*cells_radius:
                                ny = y + dy
                                nx = x + dx
                                if 0 <= nx < width and 0 <= ny < height:
                                    n_idx = ny * width + nx
                                    new_map[n_idx] = 100  # Segna come muro

        return new_map

    def map_callback(self, msg):
        self.get_logger().info(f"Mappa ricevuta: {msg.info.width}x{msg.info.height}")
        self.map_info = msg.info
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # --- MODIFICA QUI ---
        # Definisci il raggio del tuo robot (o un po' di più per sicurezza)
        # Esempio: Robot largo 40cm -> raggio 0.20 -> metti 0.25 o 0.30 di margine
        SAFETY_RADIUS = 0.20

        self.get_logger().info("Gonfiaggio mappa in corso...")
        self.map_data = self.inflate_map(msg.data, msg.info.width, msg.info.height, SAFETY_RADIUS)
        self.get_logger().info("Mappa gonfiata pronta per A*.")

    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def goal_callback(self, msg):
        self.get_logger().info("Goal ricevuto!")
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        if self.map_data is None:
            self.get_logger().warning("Manca la mappa!")
        if self.robot_pose is None:
            self.get_logger().warning("Manca la posa del robot!")
        if self.map_data is not None and self.robot_pose is not None:
            self.pianifica()

    # Unity = mondo reale  [metri] -> Coordinate float: x = 10.5, y = 3.2
    # ROS2  = mondo logico [pixel] -> Coordinate int  : cella[framebuffer]

    def world_to_grid(self, wx, wy):
        if self.resolution == 0:
            return None
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return Luogo(-1, gx, gy)

    def grid_to_world(self, gx, gy):
        wx = (gx * self.resolution) + self.origin_x
        wy = (gy * self.resolution) + self.origin_y
        return (wx, wy)
    """
    def pianifica(self):
        self.get_logger().info("Avvio A*...")
        start_grid = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        goal_grid  = self.world_to_grid(self.goal_pose[0], self.goal_pose[1])
        if not (0 <= start_grid.x < self.map_info.width) or \
           not (0 <= start_grid.y < self.map_info.height):
            self.get_logger().error("Robot fuori dalla mappa!")
            return
        agente = AgentAStarGrid(start_grid, goal_grid, self.map_data,
                                self.map_info.width, self.map_info.height)
        path_grid = agente.bfs()
        if path_grid:
            self.get_logger().info(f"Percorso trovato! Celle: {len(path_grid)}")
            ros_path = Path()
            ros_path.header.frame_id = "map"
            ros_path.header.stamp = self.get_clock().now().to_msg()
            path_world = []
            log_string = "PATH (Metri):\n"
            for i, luogo in enumerate(path_grid):
                wx, wy = self.grid_to_world(luogo.x, luogo.y)
                path_world.append((wx, wy))
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0
                ros_path.poses.append(pose)
                # --- LOGGATA ROZZA ---
                # Stampo solo ogni 5 punti per non intasare la console, oppure inizio e fine
                if i == 0 or i == len(path_grid) - 1 or i % 5 == 0:
                    log_string += f"[{i}] x={wx:.2f}, y={wy:.2f}\n"
            self.path_pub.publish(ros_path)
            self.get_logger().info(log_string)
            self.get_logger().info("Path pubblicato su topic /pianifica_path")
        else:
            self.get_logger().error("Nessun percorso trovato.")

    def simplify_path(self, path_grid):
        '''
        Prende una lista di punti [(x,y), (x,y)...] e rimuove quelli intermedi
        se la direzione non cambia.
        '''
        if len(path_grid) < 3:
            return path_grid

        new_path = [path_grid[0]]

        # Calcoliamo la direzione del primo passo
        last_dx = path_grid[1].x - path_grid[0].x
        last_dy = path_grid[1].y - path_grid[0].y

        for i in range(2, len(path_grid)):
            curr_dx = path_grid[i].x - path_grid[i-1].x
            curr_dy = path_grid[i].y - path_grid[i-1].y

            # Se la direzione è cambiata, salviamo il punto di svolta (i-1)
            if curr_dx != last_dx or curr_dy != last_dy:
                new_path.append(path_grid[i-1])
                last_dx = curr_dx
                last_dy = curr_dy

        # Aggiungiamo sempre l'ultimo punto (il goal)
        new_path.append(path_grid[-1])

        return new_path
    """
    def simplify_path(self, path_grid):
        """
        Versione per Tuple (x, y)
        """
        if len(path_grid) < 3:
            return path_grid

        new_path = [path_grid[0]]

        # ORA USIAMO GLI INDICI: [0] per X, [1] per Y
        # path_grid[1][0] significa: prendi il punto 1, prendi la coordinata X
        last_dx = path_grid[1][0] - path_grid[0][0]
        last_dy = path_grid[1][1] - path_grid[0][1]

        for i in range(2, len(path_grid)):
            # Per comodità scompattiamo le tuple
            curr_x, curr_y = path_grid[i]
            prev_x, prev_y = path_grid[i-1]

            curr_dx = curr_x - prev_x
            curr_dy = curr_y - prev_y

            # Se la direzione è cambiata, salviamo il punto di svolta (i-1)
            if curr_dx != last_dx or curr_dy != last_dy:
                new_path.append(path_grid[i-1])
                last_dx = curr_dx
                last_dy = curr_dy

        # Aggiungiamo sempre l'ultimo punto (il goal)
        new_path.append(path_grid[-1])

        return new_path

    def pianifica(self):
        self.get_logger().info("Avvio A* (8-neigh + bbox)...")

        start_grid = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        goal_grid  = self.world_to_grid(self.goal_pose[0], self.goal_pose[1])

        start = (start_grid.x, start_grid.y)
        goal  = (goal_grid.x, goal_grid.y)

        w = self.map_info.width
        h = self.map_info.height

        # bbox: rettangolo tra start e goal + margine
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        margin = max(50, int(0.5 * max(dx, dy)))

        minx = max(0, min(start[0], goal[0]) - margin)
        maxx = min(w - 1, max(start[0], goal[0]) + margin)
        miny = max(0, min(start[1], goal[1]) - margin)
        maxy = min(h - 1, max(start[1], goal[1]) + margin)
        bbox = (minx, maxx, miny, maxy)

        path_grid = astar_8conn(self.map_data, w, h, start, goal, bbox=bbox, allow_unknown=False)

        # fallback: se nel bbox non trova, prova full map
        if path_grid is None:
            self.get_logger().warning("Nessun path nel bbox, provo full-map...")
            path_grid = astar_8conn(self.map_data, w, h, start, goal, bbox=None, allow_unknown=False)

        if not path_grid:
            self.get_logger().error("Nessun percorso trovato.")
            return

        self.get_logger().info(f"Percorso trovato! Celle: {len(path_grid)}")
        self.get_logger().info(f"{path_grid}")
        self.get_logger().info(f"{self.simplify_path(path_grid)}")

        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in path_grid:  # self.simplify_path(path_grid)
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)
        self.get_logger().info("Path pubblicato su topic /pianifica_path")

def main(args=None):
    rclpy.init(args=args)
    node = Pianifica()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
from geometry_msgs.msg import Twist  # Serve per guidare il robot
from math import atan2, sin, cos

class Pianifica(Node):
    def __init__(self):
        # ... (tutto quello che avevi prima) ...

        # 1. PUBLISHER PER COMANDARE IL ROBOT
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variabili per il Path Following
        self.current_path_world = [] # Lista di tuple (x,y) in metri
        self.current_goal_idx = 0

        # 2. TIMER DI GUIDA (es. 10 Hz)
        # Questo timer controlla il robot continuamente
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Parametri del "Pilota"
        self.Kp_linear = 0.5   # Velocità
        self.Kp_angular = 2.0  # Reattività sterzo
        self.dist_threshold = 0.10 # 10 cm di tolleranza per dire "arrivato al waypoint"

    # ... (map_callback, odom_callback, goal_callback, A* ... rimangono uguali) ...

    # Modifica alla fine di pianifica() per avviare la guida
    def pianifica(self):
        # ... (calcolo A* e path_grid come prima) ...

        if path_grid:
            # Semplifica il percorso (opzionale ma consigliato)
            # Converti in metri
            self.current_path_world = []
            for node in path_grid: # (o path semplificato)
                wx, wy = self.grid_to_world(node.x, node.y)
                self.current_path_world.append((wx, wy))

            # Resetta l'indice per iniziare dal primo punto
            self.current_goal_idx = 0
            self.get_logger().info("Inizio inseguimento percorso!")

    # 3. IL PILOTA AUTOMATICO
    def control_loop(self):
        # Se non ho un percorso o non so dove sono, mi fermo
        if not self.current_path_world or self.robot_pose is None:
            return

        # Ho finito i waypoint?
        if self.current_goal_idx >= len(self.current_path_world):
            self.stop_robot()
            self.current_path_world = [] # Pulisco
            self.get_logger().info("Percorso completato! Arrivato.")
            return

        # Target corrente
        tx, ty = self.current_path_world[self.current_goal_idx]
        rx, ry = self.robot_pose # (x, y) - Nota: serve anche theta!

        # OCCHIO: self.robot_pose in odom_callback deve salvare anche THETA!
        # Nel tuo codice attuale salvavi solo x,y. Devi estrarre lo yaw dal quaternione.
        r_theta = self.robot_theta # <--- Assicurati di averlo (vedi sotto)

        # Calcoli
        dx = tx - rx
        dy = ty - ry
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)

        # Errore angolare normalizzato (-PI a +PI)
        angle_error = angle_to_goal - r_theta
        while angle_error > 3.14159: angle_error -= 2*3.14159
        while angle_error < -3.14159: angle_error += 2*3.14159

        # Logica di controllo
        cmd = Twist()

        if distance < self.dist_threshold:
            # Waypoint raggiunto, passo al prossimo
            self.current_goal_idx += 1
        else:
            # Guida P-Controller (Proporzionale)

            # Sterzata
            cmd.angular.z = self.Kp_angular * angle_error

            # Gas: Se sono allineato vado veloce, se devo girare rallento
            # (Un semplice trucco per non derapare)
            if abs(angle_error) < 0.5: # Se l'errore è < 30 gradi circa
                cmd.linear.x = self.Kp_linear * distance
                # Limito la velocità massima (es. 0.5 m/s)
                if cmd.linear.x > 0.5: cmd.linear.x = 0.5
            else:
                cmd.linear.x = 0.0 # Mi giro da fermo se l'angolo è troppo diverso

            self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    # --- AGGIUNTA IMPORTANTE: Estrarre Theta nell'odom_callback ---
    def odom_callback(self, msg):
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Estrarre Yaw (Theta) dal quaternione
        q = msg.pose.pose.orientation
        # Formula rapida per yaw da quaternione
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_theta = atan2(siny_cosp, cosy_cosp)
"""

# cd /root/ros2_ws/ && source install/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
