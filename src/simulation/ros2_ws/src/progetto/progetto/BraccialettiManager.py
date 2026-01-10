import rclpy
from rclpy.node import Node
import numpy as np
# from scipy import linalg
import json
from std_msgs.msg import String

from progetto.SincronizzaManager import SincronizzaManager
from progetto.Specialista import Specialista

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class LinearKalmanFilter:
    def __init__(self, dt=1.0, max_trend=3.0):
        self.dim_state = 2  # Stato: [Valore, Trend]
        self.dt = dt
        self.max_trend = max_trend
        # Valore_new = Valore + Trend * dt
        # Trend_new  =          Trend
        self.F = np.array([[1, dt],  # matrice di transizione
                           [0, 1]])
        # misuriamo solo Valore (prima componente dello stato)
        self.H = np.array([[1, 0]])     # matrice di misura
        self.Q = np.array([[0.001, 0   ],   # covarianza del processo =
                           [0    , 0.01]])  # quanto fidarsi del modello fisico
        self.R = np.array([[5.0]])      # covarianza del rumore di misura
        self.P = np.eye(self.dim_state) * 100.0  # covarianza errore
        self.x = np.zeros(self.dim_state)  # stato iniziale

    def predict(self, dt=None):
        if dt is not None:
            self.dt = dt
            self.F[0, 1] = dt
        self.x = self.F @ self.x
        self.x[1] = np.clip(self.x[1], -self.max_trend, self.max_trend)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0]

    def update(self, z):
        z_arr = np.array([z])
        y = z_arr - self.H @ self.x
        PHT = self.P @ self.H.T
        S = self.H @ PHT + self.R
        if S.shape[0] == 1:
            s = S.item()
            if s <= 1e-12:
                s = 1e-12
            K = PHT / s
        else:
            raise ValueError(f"S non scalare: shape={S.shape}")
            # try:
                # c, lower = linalg.cho_factor(S, lower=True)
                # K_transpose = linalg.cho_solve((c, lower), self.H @ self.P)
                # K = K_transpose.T
            # except linalg.LinAlgError:
                # K = PHT @ np.linalg.pinv(S)
        self.x = self.x + K @ y
        self.x[1] = np.clip(self.x[1], -self.max_trend, self.max_trend)
        I = np.eye(self.dim_state)
        I_KH = I - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        return self.x[0]

    # def get_state(self):
        # return self.x[0]

class SaluteOspite:
    def __init__(self, id_ospite):
        self.id_ospite = id_ospite
        self.kf_hr   = LinearKalmanFilter(dt=1.0, max_trend=3.0)
        self.kf_pmin = LinearKalmanFilter(dt=1.0, max_trend=4.0)
        self.kf_pmax = LinearKalmanFilter(dt=1.0, max_trend=4.0)
        self.last_valid_time = {
            'hr':   None,
            'pmin': None,
            'pmax': None
        }
        self.is_initialized = {'hr': False, 'pmin': False, 'pmax': False}
        self.outlier_count  = {'hr': 0,     'pmin': 0,     'pmax': 0}
        self.last_packet_time = None

class BraccialettiManager(Node):
    def __init__(self):
        super().__init__('BraccialettiManager')
        self.MAX_OUTLIERS = 5
        self.RESET_TIMEOUT = 10.0
        self.DT_REF = 0.1
        self.thresholds = {  # in un dt
            'hr':   30.0,  # bpm
            'pmin': 20.0,  # mmHg
            'pmax': 25.0   # mmHg
        }
        self.cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            String,
            '/unity/health_raw',
            self.listener_callback,
            10,
            callback_group=self.cb_group
        )
        self.pub = self.create_publisher(
            String,
            '/health_filtered',
            10
        )
        self.ospiti = {}
        self.sincro = SincronizzaManager(self)
        self.specialista = Specialista()
        self.get_logger().info('BraccialettiManager avviato.')
        self.timer = self.create_timer(60.0, self.gestione_periodica_salute)

    def get_or_create_ospite(self, ospite_id):
        if ospite_id not in self.ospiti:
            self.get_logger().info(f"Nuovo ospite rilevato: {ospite_id}")
            self.ospiti[ospite_id] = SaluteOspite(ospite_id)
        return self.ospiti[ospite_id]

    def handle_single_signal(self, ospite, key, raw_val, is_valid, kf, dt, now):
        if ospite.last_valid_time[key] is not None:
            time_gap = (now - ospite.last_valid_time[key]).nanoseconds / 1e9
            if time_gap > self.RESET_TIMEOUT:
                if ospite.is_initialized[key]:
                    self.get_logger().warn(f"[{ospite.id_ospite}][{key}] Timeout ({time_gap:.1f}s): Reset del filtro (Braccialetto rimosso?)")
                    ospite.is_initialized[key] = False
                    ospite.outlier_count[key] = 0
                    kf.P = np.eye(kf.dim_state) * 100.0
                ospite.last_valid_time[key] = None
        if not is_valid:
            if ospite.is_initialized[key]:
                return kf.predict(dt)
            else:
                return None
        ospite.last_valid_time[key] = now
        return self.process_signal(ospite, key, raw_val, kf, dt)

    def process_signal(self, ospite, key, raw_val, kf, dt):
        if not ospite.is_initialized[key]:
            kf.x[0] = raw_val
            kf.x[1] = 0.0
            kf.P = np.eye(kf.dim_state) * 100.0
            ospite.is_initialized[key] = True
            ospite.outlier_count[key] = 0
            return raw_val
        pred_val = kf.predict(dt)
        error = abs(raw_val - pred_val)
        scale = dt / self.DT_REF
        scale = min(max(1.0, scale), 10.0)
        thr = self.thresholds[key] * scale
        if error < thr:  # Caso normale (misura valida)
            clean_val = kf.update(raw_val)
            ospite.outlier_count[key] = 0
            return clean_val
        else:  # Anomalia
            ospite.outlier_count[key] += 1
            if ospite.outlier_count[key] < self.MAX_OUTLIERS:  # temporanea
                self.get_logger().warn(f"[{ospite.id_ospite}][{key}] Outlier scartato: {raw_val:.1f} (atteso: {pred_val:.1f})")
                return pred_val
            else:  # persistente
                self.get_logger().warn(f"[{ospite.id_ospite}][{key}] RESET FILTRO: Cambio stato rilevato a {raw_val:.1f}")
                kf.x[0] = raw_val
                kf.x[1] = 0.0
                kf.P = np.eye(kf.dim_state) * 100.0
                ospite.outlier_count[key] = 0
                return raw_val

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Errore parsing JSON: {e}")
            return
        ospite_id = str(data.get("id", "")).strip()
        if not ospite_id:
            self.get_logger().warn("JSON senza campo 'id': impossibile associare a un ospite, scarto.")
            return
        ospite = self.get_or_create_ospite(ospite_id)
        if ospite.last_packet_time is None:
            dt = 0.1
        else:
            dt = (current_time - ospite.last_packet_time).nanoseconds / 1e9
            if dt <= 0.0:
                dt = 1.0
        ospite.last_packet_time = current_time
        raw_hr   = float(data.get("hr", 0.0))
        raw_pmin = float(data.get("pmin", 0.0))
        raw_pmax = float(data.get("pmax", 0.0))
        hr_valid   = raw_hr   > 30.0
        pmin_valid = raw_pmin > 10.0
        pmax_valid = raw_pmax > 10.0
        clean_hr   = self.handle_single_signal(ospite, 'hr',   raw_hr,   hr_valid,   ospite.kf_hr,   dt, current_time)
        clean_pmin = self.handle_single_signal(ospite, 'pmin', raw_pmin, pmin_valid, ospite.kf_pmin, dt, current_time)
        clean_pmax = self.handle_single_signal(ospite, 'pmax', raw_pmax, pmax_valid, ospite.kf_pmax, dt, current_time)
        # self.get_logger().info(f"{clean_hr} {clean_pmin} {clean_pmax}")
        if clean_hr is None or clean_pmin is None or clean_pmax is None:
            self.get_logger().debug("Dati insufficienti o braccialetto rimosso")
            return
        output_data = {
            "id": ospite_id,
            "hr":   round(clean_hr,   2),
            "pmin": round(clean_pmin, 2),
            "pmax": round(clean_pmax, 2),
            "timestamp": current_time.nanoseconds / 1e9  # s
        }
        out_msg = String()
        out_msg.data = json.dumps(output_data)
        self.pub.publish(out_msg)

    def gestione_periodica_salute(self):
        self.get_logger().info("gestione_periodica_salute()")
        # TODO:
        # - Ogni minuto scrivere su output_data Neo4j!
        # - Aggiornare l'ontologia.
        # - SpiegamiTutto.
        # - Stampare lista di tutti coloro che sono in StatoChiamareSpecialista.
        # - if len(lista) != 0:
        #       for persona in lista:
        if len(self.ospiti) != 0:
            self.specialista.chiama(self, "medico", "ospite", "assiomi ritornati da spiegami tutto (es. bpm alti)")
        # - Stampare lista di tutti coloro che sono in StatoAllerta.
        # - if len(lista) != 0:
        #       for persona in lista[1:]:
        #           InteragisciScenarioC(persona, "NOTIFICA_SPECIALISTA") -> non bloccante
        #       InteragisciScenarioC(lista[0], "INIZIO") -> bloccante

def main(args=None):
    rclpy.init(args=args)
    node = BraccialettiManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
