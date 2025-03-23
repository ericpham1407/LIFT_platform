
import numpy as np
import casadi as ca

import threading
import queue
import multiprocessing as mp
import time

from optitrack_client.NatNetClient import NatNetClient

class VelocityFilter:
    def __init__(self, mocap_data_queue, body_id=None, print_method=print):
        self.mocap_data_queue = mocap_data_queue
        self.body_id = body_id
        self.print_method = print_method

        self.velocity_estimator_fn = None
        self.velocity_filter_thread = None

        self.last_data = dict()

        self.measurement_data_queue = mp.Queue()

        # exponential filter of estimates
        self.a_w = 1/5
        self.a_v = 1/2
        self.a_aa = 1/5
        self.a_a = 1/2
        
        self.max_time = 0
    
    def run(self):
        while True:
            if not self.mocap_data_queue.empty():
                # Get data from queue
                mocap_data = self.mocap_data_queue.get()
                # Initialize last data for finite difference
                for body in mocap_data.rigid_body_data.rigid_body_list:
                    if self.body_id is not None:
                        if body.id_num == self.body_id:
                            x = np.array(body.pos)
                            q = np.array(body.rot)
                            self.last_data[body.id_num] = dict(id=body.id_num,
                                                                t=mocap_data.suffix_data.timestamp, 
                                                                x=x, 
                                                                q=q, 
                                                                v=np.zeros(3), 
                                                                vb=np.zeros(3),
                                                                w=np.zeros(3),
                                                                a=np.zeros(3),
                                                                ab=np.zeros(3),
                                                                aa=np.zeros(3))
                            break
                    else:
                        x = np.array(body.pos)
                        q = np.array(body.rot)
                        self.last_data[body.id_num] = dict(id=body.id_num,
                                                            t=mocap_data.suffix_data.timestamp, 
                                                            x=x, 
                                                            q=q, 
                                                            v=np.zeros(3), 
                                                            vb=np.zeros(3),
                                                            w=np.zeros(3),
                                                            a=np.zeros(3),
                                                            ab=np.zeros(3),
                                                            aa=np.zeros(3))
                break

        # self.velocity_filter_thread = threading.Thread(target=self._velocity_filter)
        self.velocity_filter_thread = mp.Process(target=self._velocity_filter, args=(self.mocap_data_queue, self.measurement_data_queue))
        self.velocity_filter_thread.start()

    def _velocity_filter(self, in_q, out_q):
        while True:
            if not in_q.empty():
                # t = time.time()
                # self.print_method(f'Input queue size: {in_q.qsize()}')
                # Get data from queue
                # while not in_q.empty():
                #     mocap_data = in_q.get()
                mocap_data = in_q.get()
                # Estimate velocities
                for body in mocap_data.rigid_body_data.rigid_body_list:
                    if self.body_id:
                        if body.id_num == self.body_id:
                            dt = mocap_data.suffix_data.timestamp - self.last_data[body.id_num]['t']
                            x = np.array(body.pos)
                            q = np.array(body.rot)
                            data = self._estimate_velocity(dt, x, q, self.last_data[body.id_num])
                            data['t'] = mocap_data.suffix_data.timestamp
                            data['id'] = body.id_num
                            out_q.put(data)
                            self.last_data[body.id_num] = data
                            break
                    else:
                        dt = mocap_data.suffix_data.timestamp - self.last_data[body.id_num]['t']
                        x = np.array(body.pos)
                        q = np.array(body.rot)
                        data = self._estimate_velocity(dt, x, q, self.last_data[body.id_num])
                        data['t'] = mocap_data.suffix_data.timestamp
                        data['id'] = body.id_num
                        out_q.put(data)
                        self.last_data[body.id_num] = data
                # time.sleep(1e-5)
                # print(f'Estimation thread time: {time.time()-t}')
                
    def get_measurement_data(self):
        if not self.measurement_data_queue.empty():
            # print(f'Output queue size: {self.measurement_data_queue.qsize()}')
            while not self.measurement_data_queue.empty():
                measurement_data = self.measurement_data_queue.get()
            # measurement_data = self.measurement_data_queue.get()
            return measurement_data
        else:
            return None
    
    def _estimate_velocity(self, dt, x, q, dp):
        # Unpack data from previous time step
        xp  = dp['x']
        qp  = dp['q']
        vp  = dp['v'] # Inertial frame velocities
        vbp = dp['vb'] # Body frame velocities
        wp  = dp['w']
        ap  = dp['a'] # Inertial frame accelerations
        abp = dp['ab'] # Body frame accelerations
        aap = dp['aa']

        qi, qj, qk, qr = q
        R = np.array([[1-2*qj**2 - 2*qk**2, 2*(qi*qj - qk*qr),    2*(qi*qk + qj*qr)  ],
                      [2*(qi*qj + qk*qr),   1-2*qi**2 - 2*qk**2,  2*(qj*qk - qi*qr)  ],
                      [2*(qi*qk - qj*qr),   2*(qj*qk + qi*qr),    1-2*qi**2 - 2*qj**2]])

        # Inertial frame velocity estimate
        v = (x - xp) / dt
        v_est = self.a_v * v + (1-self.a_v) * vp
        # Inertial frame acceleration estimate
        a = (v - vp) / dt
        a_est = self.a_a * a + (1-self.a_a) * ap

        # Body frame velocity estimate
        vb = R.T @ v
        vb_est = self.a_v * vb + (1-self.a_v) * vbp
        # Body frame acceleration estimate
        ab = R.T @ a
        ab_est = self.a_a * ab + (1-self.a_a) * abp

        # Angular velocity estimate
        qpi, qpj, qpk, qpr = qp
        A = np.array([[ qpr,-qpk, qpj],
                      [ qpk, qpr,-qpi],
                      [-qpj, qpi, qpr],
                      [-qpi,-qpj,-qpk]])
        b = (q - qp) / dt
        w = np.linalg.pinv(A) @ b
        w_est = self.a_w * w + (1-self.a_w) * wp
        # Angular accleration estimate
        aa = (w - wp) / dt
        aa_est = self.a_aa * aa + (1-self.a_aa) * aap

        data = dict(x=x, q=q, v=v_est, vb=vb_est, a=a_est, ab=ab_est, w=w_est, aa=aa_est)
        
        return data

if __name__ == '__main__':
    streaming_client = NatNetClient()
    streaming_client.set_use_multicast(True)
    streaming_client.set_print_level(0)

    velocity_filter = VelocityFilter(streaming_client.mocap_data_queue, body_id=2)

    streaming_client.run()
    velocity_filter.run()
    last_time = time.time()

    while True:
        measurement_data = velocity_filter.get_measurement_data()
        if measurement_data:
            t = time.time()
            dt = t - last_time
            last_time = t
            print(dt)
        time.sleep(1e-6)
            
        