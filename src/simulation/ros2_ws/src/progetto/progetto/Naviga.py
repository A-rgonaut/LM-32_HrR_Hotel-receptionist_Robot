import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

from math import sqrt, atan2, cos, sin, pi
import array

from progetto.Pianifica import astar_8conn

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from rclpy.time import Time

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
        self.goal_theta = None

        self.current_path_world = [] # Lista di waypoint (x, y)
        self.current_goal_idx = 0

        # --- Parametri Controllo ---
        self.Kp_linear = 0.5
        self.Kp_angular = 1.0
        self.dist_threshold = 0.08 #  cm di tolleranza
        self.dist_threshold2 = 0.3 #  cm di tolleranza

        self.sum_angular_error = 0.0  # Accumulatore per l'errore (Termine Integrale)
        self.Ki_angular = 0.05        # Guadagno Integrale (piccolo ma costante)
        self.prev_angle_error = 0.0   # Per resettare l'integrale se cambia il target
        self.state_rotating = True

        self.pub_cmd_vel = self.nodo.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.nodo.create_publisher(Path, '/pianifica_path', 10)

        self.nodo.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        self.front_min_dist = float('inf')
        self.avoid_multiplier = 0.0
        self.nodo.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.is_base_target=False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.nodo)

        self.nodo.get_logger().info("Naviga...")

        self.timer = self.nodo.create_timer(0.1, self.control_loop)

    def esegui_retromarcia(self):
        import time
        self.nodo.get_logger().info("Eseguo retromarcia...")
        cmd = Twist()
        cmd.linear.x = -0.15
        cmd.angular.z = 0.0
        for _ in range(15):
            self.pub_cmd_vel.publish(cmd)
            time.sleep(0.1)
        self.stop_robot()


    # --- 1. LOGICA DI CONTROLLO (Chiamata dall'Arbitraggio) ---
    def control_loop(self):
        # Prima cosa: aggiorna la posa
        if not self.update_pose():
            return  # Se non so dove sono, non mi muovo
        if self.map_data is None:
            return
        # Se ho una destinazione ma non ho ancora un percorso, PIANIFICO
        if not self.current_path_world and self.nodo.destinazione_target and not self.nodo.raggiunta_destinazione:
            self.pianifica()
            return
        if not self.current_path_world:
            return

        # --- GESTIONE FINE PERCORSO ---
        # Controllo se siamo all'ultimo punto PRIMA di fare calcoli
        if self.current_goal_idx >= len(self.current_path_world):
            if self.goal_theta is not None:
                angle_error = self.goal_theta - self.robot_theta
                while angle_error > pi:
                    angle_error -= 2 * pi
                while angle_error < -pi:
                    angle_error += 2 * pi
                if abs(angle_error) > 0.05:
                    cmd = Twist()
                    cmd.angular.z = max(min(angle_error, 0.4), -0.4)
                    self.pub_cmd_vel.publish(cmd)
                    return

            self.stop_robot()
            self.nodo.get_logger().info("Naviga: Destinazione raggiunta!")
            if self.is_base_target:
                self.esegui_retromarcia()
                self.esegui_retromarcia()
            self.nodo.raggiunta_destinazione = True
            # Invece di chiamare il comportamento_attivo (che è Naviga),
            # svegliamo solo il comportamento che ci ha chiamati (B o C).
            target = self.nodo.comportamento_precedente
            if target in ["InteragisciScenarioB", "InteragisciScenarioC"]:
                self.nodo.get_logger().info(f"Sveglio lo {target}...")
                self.nodo.comportamenti[target].esegui("")
            self.current_path_world = []
            return

        # --- INSEGUIMENTO WAYPOINT ---
        tx, ty = self.current_path_world[self.current_goal_idx]
        rx, ry = self.robot_pose

        dx = tx - rx
        dy = ty - ry
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)

        FRONT_BLOCKED_DIST = 0.2
        if not self.state_rotating and self.front_min_dist < FRONT_BLOCKED_DIST and self.front_min_dist < (distance - 0.2):
            self.nodo.get_logger().warning("Percorso bloccato -> Ripianifico!")
            self.mark_blocked_ahead()
            self.esegui_retromarcia()
            self.reset()
            return

        # Errore angolare normalizzato (-PI a +PI)
        angle_error = angle_to_goal - self.robot_theta
        while angle_error > pi:
            angle_error -= 2 * pi
        while angle_error < -pi:
            angle_error += 2 * pi

        # --- LOGICA CAMBIO WAYPOINT ---
        # Se siamo vicini al waypoint corrente
        if distance < self.dist_threshold:
            self.current_goal_idx += 1
            # Non forziamo lo stop. Se il prossimo punto è dritto, continuerà fluido.
            # Se è una curva a 90°, l'if sulla "TOLLERANZA_START_ROT" qui sotto
            # lo fermerà nel prossimo ciclo del timer (0.1s dopo).
            return

        # Reset dell'integrale se cambia segno o cambio drastico
        if (angle_error * self.prev_angle_error < 0) or (abs(angle_error - self.prev_angle_error) > 1.0):
             self.sum_angular_error = 0.0
        self.prev_angle_error = angle_error

        cmd = Twist()

        # --- PARAMETRI ---
        MAX_ANG_VEL = 0.4
        TOLLERANZA_STOP_ROT = 0.10   # gradi
        TOLLERANZA_START_ROT = 0.35

        # --- MACCHINA A STATI ---
        if self.state_rotating:
            # Smetto di ruotare se sono allineato
            if abs(angle_error) < TOLLERANZA_STOP_ROT:
                self.state_rotating = False
                self.sum_angular_error = 0.0
        else:
            # Mi fermo per ruotare SOLO se l'errore è grande (curva secca)
            if abs(angle_error) > TOLLERANZA_START_ROT:
                self.state_rotating = True
                self.sum_angular_error = 0.0

        # --- ESECUZIONE ---
        if self.state_rotating:
            # --- RUOTA SUL POSTO ---
            cmd.linear.x = 0.0

            sign = 1.0 if angle_error > 0 else -1.0
            raw_rot = (self.Kp_angular * angle_error) + (self.Ki_angular * self.sum_angular_error)

            min_push = 0.15
            angular_speed = raw_rot + (min_push * sign)

            if abs(angular_speed) > MAX_ANG_VEL:
                angular_speed = MAX_ANG_VEL * sign

            cmd.angular.z = angular_speed

        else:
            cmd.linear.x = min(self.Kp_linear * distance, 0.2)

            # Clamp sull'errore per evitare scatti violenti mentre si guida
            correction = angle_error * 1.5 # Kp dinamico per la guida

            AVOID_DIST = 0.6  # Distanza a cui inizio a schivare

            # Schivo SOLO se l'ostacolo è più vicino del mio obiettivo (-0.15 margine)
            if self.front_min_dist < AVOID_DIST and self.front_min_dist < (distance - 0.15):
                # Più sono vicino, più spingo via
                push = 1.0 * (AVOID_DIST - self.front_min_dist)
                # Applico direzione calcolata dal Laser
                correction += (push * self.avoid_multiplier)
                self.nodo.get_logger().info("Schivo ostacolo senza ripianificare...", throttle_duration_sec=1.0)

            if abs(angle_error) < 0.03:
                 # Se non sto schivando e l'errore è piccolo, vado dritto
                 if self.front_min_dist > AVOID_DIST:
                     correction = 0.0

            # Clamp finale
            cmd.angular.z = max(min(correction, 0.6), -0.6)

        self.pub_cmd_vel.publish(cmd)

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())

    # --- 2. PIANIFICAZIONE (A*) ---
    def pianifica(self):
        self.nodo.raggiunta_destinazione = False
        start_pose = self.robot_pose
        goal_pose = self.nodo.destinazione_target
        self.goal_theta = goal_pose[2] if len(goal_pose) == 3 else None

        if not start_pose or not goal_pose:
            return

        dist_attuale = sqrt((start_pose[0] - goal_pose[0])**2 + (start_pose[1] - goal_pose[1])**2)
        if dist_attuale < self.dist_threshold2:
            self.nodo.get_logger().info("Naviga: Già a destinazione (sotto soglia).")
            self.nodo.raggiunta_destinazione = True
            return

        self.nodo.get_logger().info(f"Naviga: Calcolo percorso da {start_pose} a {goal_pose}...")

        # Coordinate Griglia
        sx, sy = self.world_to_grid(start_pose[0], start_pose[1])
        gx, gy = self.world_to_grid(goal_pose[0], goal_pose[1])

        w = self.map_info.width
        h = self.map_info.height

        # Se il robot si trova su un pixel "nero" (causa inflazione),
        # cerchiamo il primo pixel libero in un raggio di 8 celle (circa 40cm)
        start_idx = sy * w + sx
        if self.map_data[start_idx] == 100:
            self.nodo.get_logger().warn(f"Start {sx},{sy} bloccato dall'inflazione! Cerco via di fuga...")
            found_free = False
            raggio_max = 8  # Aumenta se la risoluzione è molto alta
            for r in range(1, raggio_max + 1):
                # Controlla un quadrato attorno al robot
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        nx = sx + dx
                        ny = sy + dy
                        # Controllo limiti mappa
                        if 0 <= nx < w and 0 <= ny < h:
                            if self.map_data[ny * w + nx] != 100:
                                # Trovato un punto libero! Aggiorno lo start
                                sx, sy = nx, ny
                                found_free = True
                                break
                    if found_free:
                        break
                if found_free:
                    break
            if found_free:
                self.nodo.get_logger().info(f"Nuovo start libero trovato: {sx},{sy}")
            else:
                self.nodo.get_logger().error("Impossibile trovare punto libero vicino al robot!")
                return # Fallimento reale

        path_grid = []
        self.is_base_target = abs(goal_pose[0] - 10.0) < 0.2 and abs(goal_pose[1] - 11.0) < 0.2
        if self.is_base_target:
            self.nodo.get_logger().info("Naviga: is_base_target")
            mx, my = self.world_to_grid(6.0, 8.0)
            path_grid = astar_8conn(self.map_data, w, h, (sx, sy), (mx, my), bbox=None, allow_unknown=False)
            if path_grid:
                path_grid.append( self.world_to_grid(10.0, 10.0))
                path_grid.append((gx, gy))
            else:
                 self.nodo.get_logger().error("Fallito calcolo is_base_target")
        else:

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

            # --- FORZO LA ROTAZIONE ---
            # Dico al robot che sta già ruotando.
            # Così nel prossimo loop salta il check del muro (che ha 'if not state_rotating')
            # e inizia subito a girarsi verso la nuova via di fuga.
            self.state_rotating = True
            self.sum_angular_error = 0.0
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

        SAFETY_RADIUS = 0.5 + 0.3  # Metri di sicurezza attorno agli ostacoli
        self.map_data = self.inflate_map(msg.data, msg.info.width, msg.info.height, SAFETY_RADIUS)

    def inflate_map(self, data, width, height, robot_radius_meters):
        cells_radius = int(robot_radius_meters / self.resolution)
        if cells_radius <= 0:
            return list(data)

        # Copia mutabile della mappa
        new_map = list(data)

        # 1. OTTIMIZZAZIONE: Pre-calcoliamo il "pennello" (gli offset del cerchio)
        # Lo facciamo una volta sola, non per ogni pixel!
        offsets = []
        r2 = cells_radius * cells_radius
        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                if dx*dx + dy*dy <= r2:
                    offsets.append(dy * width + dx) # Salviamo l'offset lineare (più veloce)

        # 2. OTTIMIZZAZIONE: Troviamo SOLO gli indici degli ostacoli esistenti
        # List comprehension è molto più veloce dei for loop espliciti in Python
        obstacle_indices = [i for i, val in enumerate(data) if val == 100]

        # 3. Applichiamo l'espansione solo dove serve
        len_map = len(new_map)

        for idx in obstacle_indices:
            # Per ogni ostacolo, applichiamo gli offset pre-calcolati
            for offset in offsets:
                neighbor_idx = idx + offset

                # Controllo rapido dei bound (senza fare calcoli x, y costosi)
                # Nota: questo controllo è approssimato per velocità (ignora wrap-around sui bordi orizzontali)
                # ma per il path planning locale è solitamente accettabile e MOLTO più veloce.
                if 0 <= neighbor_idx < len_map:
                    new_map[neighbor_idx] = 100

        return new_map

    def scan_callback(self, msg):
        center = len(msg.ranges) // 2
        # Prendo un cono stretto centrale (+/- 15 gradi circa)
        left_window = msg.ranges[center : center + 15]
        right_window = msg.ranges[center - 15 : center]

        # Trovo la distanza minima valida per lato
        l_min = min((r for r in left_window if msg.range_min < r < msg.range_max), default=float('inf'))
        r_min = min((r for r in right_window if msg.range_min < r < msg.range_max), default=float('inf'))

        self.front_min_dist = min(l_min, r_min)

        # Se l'ostacolo è a DESTRA (r_min < l_min), devo spingere a SINISTRA (+1.0)
        # Se l'ostacolo è a SINISTRA, devo spingere a DESTRA (-1.0)
        if self.front_min_dist < float('inf'):
            self.avoid_multiplier = 1.0 if r_min < l_min else -1.0
        else:
            self.avoid_multiplier = 0.0

    def mark_blocked_ahead(self):
        if self.map_data is None or self.robot_pose is None:
            return

        res = self.resolution if self.resolution > 0 else 0.05
        w_map = self.map_info.width
        h_map = self.map_info.height

        rx, ry = self.robot_pose
        theta = self.robot_theta

        # 1. Calcolo il centro del muro (cm davanti al robot)
        DIST_AVANTI = 0.8
        cx = rx + DIST_AVANTI * cos(theta)
        cy = ry + DIST_AVANTI * sin(theta)

        # Converto il centro in coordinate griglia
        gx, gy = self.world_to_grid(cx, cy)

        # 2. Configurazione Dimensioni
        LARGHEZZA_METRI = 2.5  # Quanto è lungo il muro
        SPESSORE_METRI = 0.20  # Spessore (20cm)

        half_len_px = int((LARGHEZZA_METRI / 2) / res)
        half_thick_px = int((SPESSORE_METRI / 2) / res)
        # Assicuro almeno 1 pixel di spessore
        if half_thick_px < 1: half_thick_px = 1

        updated = 0

        # 3. DECISIONE ORIENTAMENTO (Snap to Grid)
        # Se |cos| > |sin|, il robot si muove più in orizzontale -> Muro VERTICALE
        # Se |sin| > |cos|, il robot si muove più in verticale -> Muro ORIZZONTALE

        if abs(cos(theta)) > abs(sin(theta)):
            # --- MURO VERTICALE (Nord-Sud) ---
            # Varia Y (Lunghezza), Fisso X (Spessore)
            for dy in range(-half_len_px, half_len_px + 1):
                for dx in range(-half_thick_px, half_thick_px + 1):
                    nx = gx + dx
                    ny = gy + dy
                    if 0 <= nx < w_map and 0 <= ny < h_map:
                        self.map_data[ny * w_map + nx] = 100
                        updated += 1
            type_str = "VERTICALE (N-S)"

        else:
            # --- MURO ORIZZONTALE (Ovest-Est) ---
            # Varia X (Lunghezza), Fisso Y (Spessore)
            for dx in range(-half_len_px, half_len_px + 1):
                for dy in range(-half_thick_px, half_thick_px + 1):
                    nx = gx + dx
                    ny = gy + dy
                    if 0 <= nx < w_map and 0 <= ny < h_map:
                        self.map_data[ny * w_map + nx] = 100
                        updated += 1
            type_str = "ORIZZONTALE (W-E)"

        self.nodo.get_logger().warn(f"Disegnato muro {type_str} di 3m davanti al robot.")

    def update_pose(self):
        try:
            # Dove si trova il robot (base_link) rispetto alla mappa (map)?
            from rclpy.time import Time
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            self.robot_pose = (t.transform.translation.x, t.transform.translation.y)
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.robot_theta = atan2(siny_cosp, cosy_cosp)
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.nodo.get_logger().warning('In attesa di localizzazione AMCL (map->base_link)...', throttle_duration_sec=2.0)
            return False

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
        wx = (gx * self.resolution) + (self.resolution * 0.5) + self.origin_x
        wy = (gy * self.resolution) + (self.resolution * 0.5) + self.origin_y
        return wx, wy

    def reset(self):
        self.current_path_world = []
        self.current_goal_idx = 0
        self.stop_robot()

    def esegui(self, testo):
        pass
