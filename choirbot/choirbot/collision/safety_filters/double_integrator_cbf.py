import numpy as np
import time
import qpSWIFT

from .safety_filter_strategy import SafetyFilterStrategy

class DoubleIntegratorCBF(SafetyFilterStrategy):
    '''
    Double Integrator Control Barrier Function

    Inspired by

    @article{borrmann2015control,
        title={Control barrier certificates for safe swarm behavior},
        author={Borrmann, Urs and Wang, Li and Ames, Aaron D and Egerstedt, Magnus},
        journal={IFAC-PapersOnLine},
        volume={48},
        number={27},
        pages={68--73},
        year={2015},
        publisher={Elsevier}
    }

    '''

    def __init__(self, distance: float = 0.4, gamma: float = 0.1, kappa: float = 1.0, max_acceleration: float = 0.1, max_braking_acceleration: float = 0.2):
        super().__init__()
        '''
        Args:
        - distance (float): safe distance to the obstacles
        - gamma (float): safety parameter
        - kappa (float): safety parameter
        - max_acceleration (float): maximum acceleration (in norm)
        - max_braking_acceleration (float): maximum breaking acceleration (in norm)
        '''

        self.distance = distance
        self.gamma = gamma
        self.kappa = kappa
        self.max_acceleration = max_acceleration
        self.max_braking_acceleration = max_braking_acceleration

        self.min_barrier_values = []
        self.starting_time = time.time()

    def check_barrier(self):
        for obstacle in self.obstacles:
            delta_p = self.state[:self.state_dim//2] - obstacle[:self.state_dim//2]
            delta_v = self.state[self.state_dim//2:] - obstacle[self.state_dim//2:]
            delta_a = 2 * self.max_acceleration

            delta_p_norm = np.linalg.norm(delta_p) 

            if delta_p_norm <= self.distance:
                print("[WARN] Distance NOT Safe")
                return False

            h_ij = np.dot(delta_p.T, delta_v) / (delta_p_norm) + np.sqrt(self.kappa * delta_a * (delta_p_norm - self.distance))

            if h_ij < 0:
                print("[WARN] Barrier Violated")
                return False
            
        return True

    def hybrid_braking_controller(self):
        '''
        Hybrid Braking Controller. If the barrier is violated, the controller applies a braking input in the form

        u = - max_braking_acceleration * velocity / || velocity ||

        Returns:
        - u (list): control input
        '''
        velocity = self.state[self.state_dim//2:]
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > 1e-5:
            u_brake = -self.max_braking_acceleration*velocity/velocity_norm
            print(f'Hybrid Braking Controller ON | Braking input applied: {u_brake.flatten()}')
            return -self.max_braking_acceleration*velocity/velocity_norm
        else:
            return np.zeros(velocity.shape)


    def compute_obstacle_constraints(self):
        '''
        This function computes the constraints for the CBF controller,
        namely, for each obstacle in the list, it computes
        
            A u <= b

        where, the CBF is defined as:
            h_ij = <delta_p, delta_v> / || delta_p || + sqrt(kappa * delta_a * (|| delta_p || - distance))
        
        and the constraints are:
            A = - delta_p.T
            b = gamma * h_ij^3 * || delta_p || - <delta_v, delta_p>^2 / || delta_p ||^2 + || delta_v ||^2 + kappa * delta_a * <delta_v, delta_p> / sqrt(2*kappa * delta_a * (|| delta_p || - distance))

        '''

        A = []
        b = []

        for obstacle in self.obstacles:
            delta_state = self.state - obstacle
            delta_p = delta_state[:self.state_dim//2]
            delta_v = delta_state[self.state_dim//2:]
            delta_a = 2 * self.max_acceleration

            delta_p_norm = np.linalg.norm(delta_p) 
            delta_v_norm = np.linalg.norm(delta_v) 

            h_ij = np.dot(delta_p.T, delta_v) / (delta_p_norm) + np.sqrt(self.kappa * delta_a * (delta_p_norm - self.distance))                
            A_ij = - delta_p.T
            b_ij = self.gamma * h_ij**3 * delta_p_norm \
                    - np.dot(delta_v.T, delta_p)**2 / delta_p_norm**2 \
                    + delta_v_norm**2 \
                    + self.kappa * delta_a * np.dot(delta_v.T, delta_p) / np.sqrt(2*self.kappa * delta_a * (delta_p_norm - self.distance))
            b_ij *= 0.5
            
            A.append(A_ij)
            b.append(b_ij)

        # reshape A and b
        A = np.array(A, dtype=float).reshape(-1, self.input_dim)
        b = np.array(b, dtype=float).reshape(-1, 1)

        return A, b
   

    def compute_safe_input(self):
        '''
        Compute a safe input for a double integrator system using a Control Barrier Function (CBF) with the qpSWIFT solver.

        minimize    || u - u_{ref}||^2
        u\in R^n

        subject to  A_bar u <= b_bar
                    || u ||_{inf} <= c_bar

        mapped into

        minimize    y^T y
        u\in R^n
        y\in R^n

        subject to  y = u - u_{ref}
                    -inf <= A_bar u <= b_bar
                    -c_bar  <=  u  <= c_bar

        qpSWIFT problem formulation:
        P = 2*diag([zeros(n,n), eye(n)])
        q = zeros(2*n)
        G = [[ A,                 zeros(m,n)],
             [ eye(n),            zeros(n,n)],
             [-eye(n),            zeros(n,n)]]
        h = [ b, c, c].T
        A = [eye(n), -eye(n)]
        b = [u_ref]


        Returns:
        - safe_input (list): safe control input
        '''


        if self.check_barrier():
            A_bar, b_bar = self.compute_obstacle_constraints()
            m, n = np.shape(A_bar)
            c_bar = self.max_acceleration*np.ones((n,1))
  
            P = np.zeros((2*n,2*n), dtype=float)
            P[n:,n:] = 2*np.eye(n, dtype=float)
            c = np.zeros(2*n, dtype=float)

            # Inequality constraints
            G = np.bmat([[A_bar,                    np.zeros((m,n),dtype=float)],
                         [ np.eye(n,dtype=float),   np.zeros((n,n),dtype=float)],
                         [-np.eye(n,dtype=float),   np.zeros((n,n),dtype=float)]
                         ])
            h = np.vstack([b_bar, c_bar, c_bar]).flatten()

            # Equality constraints
            A = np.bmat([np.eye(n,dtype=float), -np.eye(n,dtype=float)])
            b = self.desired_input.flatten()

            sol = qpSWIFT.run(c,h,P,G,A,b,opts = {'MAXITER':199, 'OUTPUT':2})

            safe_input = sol['sol'][:n]

            if np.any(np.abs(safe_input) >= self.max_acceleration) or np.any(np.isnan(safe_input)):
                if sol['basicInfo']['ExitFlag'] == 1:
                    print("Problem not solved: ERR1 = Factorization KKT failure")
                elif sol['basicInfo']['ExitFlag'] == 2:
                    print(f"Problem not solved: ERR2 = Reached ({sol['basicInfo']['Iterations']}) Iterations")
                elif sol['basicInfo']['ExitFlag'] == 3:
                    print("Problem not solved: ERR3 = Unknown problem in Solver")
                else:
                    print(f"Problem not solved: ERR UNKNOWN - {sol['basicInfo']}")
                safe_input = self.hybrid_braking_controller()

            # else:
            #     print(f"n_obs = {len(self.obstacles)} | u_des = {self.desired_input.flatten()} \tu_safe = {safe_input}")
        else:
            safe_input = self.hybrid_braking_controller()

        if self.obstacles:
            self.min_barrier_values.append([time.time() - self.starting_time, self.get_min_barrier_value()])

        return safe_input.flatten().tolist()
    
    def get_min_barrier_value(self):
        
        barrier_value = []

        for obstacle in self.obstacles:
            # Control Barrier Function
            # h_{ij}(x) = ||x_i - x_{obs,j}||^2 - d^2

            delta_p = self.state - obstacle
            delta_p_norm = np.linalg.norm(delta_p)
            barrier_value.append(delta_p_norm**2 - self.distance**2)
                
        return min(barrier_value)

    def get_min_barrier_value_list(self):
        return self.min_barrier_values