# Kalman Filter

## Teoria

Il filtro di Kalman è un filtro per cui si hanno: stati e osservazioni. L'obiettivo è filtrare le misurazione rumorose per avere una migliore stima che andrà a predirre il nostro filtro; si ha inoltre l'errore di processo, ovvero l'errore di input, non del tutto preciso, cui il sistema è soggetto.

x_(k) = F_(k) \dot x_(x-1) + B_(k) \dot u_(k) + w_(k), w_(k) ~ N(0,Q)

y_(k) = H_(k) \dot x_(k) + v_(k), v_(k) ~ N(0,R)

E[w_(k), v_(i)] = 0;\
E[w_(k), w_(i)] = 0, k \ne i;\
E[w_(k), w_(k)] = Q

Nel modello abbiamo due fasi:
- **fase di prediction**, sfrutta il modello per predirre lo stato.

    x\hat_(k) = F_(k) \dot x\hat_(x-1) + B_(k) \dot u_(k), x_(0) ~ N(x_(0),P_(0))

    Per un punto abbiamo tnate configurazioni incerte quanto sono le colonne della matrice di covarianza.

    C = A \Sigma A^(T) -> P_(k)^(-) = F_(k) P_(k-1) F_(k)^(t) + Q 

    il ^(-) indica che è un calcolo intermedio, l'aggiornamento della matrice di covarianza avviene nello stato di update.

- **fase di update**, cattura la misurazione, vede quanto differisce da quella predetta, verifica quanto sia affidabile in merito alla misurazione e: se il modello è abbastanza affidabile, sfruttiamo l'errore per aggiornare la stima del modello; altrimenti no.

    \epsilon_(obs) = y_(k) - H_(k) \dot x\hat_(k)^(-)

    x\hat_(k)^ = x\hat_(k)^(-) + K \dot \epsilon_(obs)

    La covarianza sulla misurazione è

    S_(k) = H_(k) P_(k)^(-) H^(T) + R

    Il guadagno dev'essere tanto più grande quanta è precisa la misurazione

    K = P_(k)^(-) H_(k)^(T) S_(k)^(-1)
    
    E quindi l'aggiornamento sarà

    P_(k) = (I - K H_(k)) P_(k)^(-)

## Pratica

kalman_filter.py

```python
    import rclpy
    from rclpy.node import Node

    class MeasurementSimulator(Node):

        def __init__(self, dt = 0.1):
            self.dim_state = 4
            self.dt = dt
            self.F.np.array([
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            self.H.np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])
            self.Q = np.eye(self.dim_state) * 0.01
            self.R = np.eye(2) * 0.1
            self.P = np.eye(self.dim_state)

        def predict(self):
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q

            return self.x[:2]

        def update(self):
            S = self.H @ self,P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg......


    def main(args=Node):

    if __name__ == "main"
```

kalman_node.py

```python
    import rclpy
    from rclpy.node import Node

    class KalmanFilterNode(Node):

        def __init__(self):
            super().__init__('kalman_filter_node')

            self.kf = LinearKalmanFilter(dt=0.1)

            self.estimated_pub = self.create_publisher(
                PoseStamped,
                '/estimated_position',
                10
            )
            
            self.marker_pub = self.create_publisher(
                Marker,
                '/kalman_markers',
                10
            )

            self.measurement_sub = self.create_subscription(
                PointStamped,
                '\gps_measurement'
                ......
            )
            

        def measurement_callback(self, msg):
            z = np.array([msg.point_x, msg.point_y])

            self.measurements.append(z.copy())

            self.kf.predict()

            estimated_pos= selk.kf.update(z)

            self.estimations.append(estimated_pos.copy())

            self.get_logger().info(
                f'Misura: {(z[0]:.2f), (z[1]:.2f)}',
                f'Stima: {(estimat......)}',
            )

        def publish_estimation(self):
            if not hasattr(self, ......)

        def publish_markers(self, measurement, estimation):

        def plot_results(self):


    def main(args=Node):

    if __name__ == "main"
```

simulator_node.py

```python
    import rclpy
    from rclpy.node import Node

    class MeasurementSimulator(Node):

        def __init__(self):


        def publish_measurement(self):


    def main(args=Node):

    if __name__ == "main"
```

Dopo aver completato le classi si eseguono gli script

I punti della triattoria verde sono più densi/circoscritti, i punti rossi invece più sparsi. A noi interessa il pattern generale.