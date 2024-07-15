import numpy as np
import time
import qpSWIFT

from .safety_filter_strategy import SafetyFilterStrategy

class SingleIntegratorCBF(SafetyFilterStrategy):
    '''
    Single Integrator Control Barrier Function

    Inspired by
    
    @article{wilson2020robotarium,
        title={The robotarium: Globally impactful opportunities, challenges, and lessons learned in remote-access, distributed control of multirobot systems},
        author={Wilson, Sean and Glotfelter, Paul and Wang, Li and Mayya, Siddharth and Notomista, Gennaro and Mote, Mark and Egerstedt, Magnus},
        journal={IEEE Control Systems Magazine},
        volume={40},
        number={1},
        pages={26--44},
        year={2020},
        publisher={IEEE}
    }

    '''

    def __init__(self, distance: float = 0.4, gamma: float = 5.0, max_velocity: float = 0.2, max_braking_velocity: float = 0.4):
        super().__init__()
        '''
        Args:
        - distance (float): safe distance to the obstacles
        - gamma (float): safety parameter
        - max_velocit (list): maximum velocity for each dimension
        '''
        self.distance = distance
        self.gamma = gamma
        self.max_velocity = max_velocity
        self.max_braking_velocity = max_braking_velocity

        self.min_barrier_values = []
        self.starting_time = time.time()

    def hybrid_braking_controller(self):
        '''
        Hybrid Braking Controller. If the barrier is violated, the controller applies a braking input in the form

        u = - 0.5*max_velocity

        Returns:
        - u (list): control input
        '''
        direction = np.zeros((self.input_dim,1))

        for obstacle in self.obstacles:
            # Control Barrier Function
            # h_{ij}(x) = ||x_i - x_{obs,j}||^2 - d^2

            delta_p = self.state - obstacle
            delta_p_norm = np.linalg.norm(delta_p)

            if delta_p_norm <= self.distance*1.5:
                direction = delta_p/delta_p_norm
            
        u_brake = self.max_braking_velocity*direction
        print(f'Hybrid Braking Controller ON | Braking input applied: {u_brake.flatten()}')
        return u_brake

    def compute_obstacle_constraints(self):
        '''
        This function computes the constraints for the CBF controller,
        namely, for each obstacle in the list, it computes
        
            A u <= b

        where, the CBF is defined as:
            h_ij = <delta_p, delta_p> - distance^2
        
        and the constraints are:
            A = - delta_p.T
            b = gamma * h_ij^3 * || delta_p || - <delta_v, delta_p>^2 / || delta_p ||^2 + || delta_v ||^2 + kappa * delta_a * <delta_v, delta_p> / sqrt(2*kappa * delta_a * (|| delta_p || - distance))

        '''

        A = []
        b = []

        for obstacle in self.obstacles:
            # Control Barrier Function
            # h_{ij}(x) = ||x_i - x_{obs,j}||^2 - d^2

            delta_p = self.state - obstacle
            delta_p_norm = np.linalg.norm(delta_p)

            h_ij = delta_p_norm**2 - self.distance**2

            A_ij = - 2 * delta_p.T
            b_ij = self.gamma * h_ij**3

            A.append(A_ij)
            b.append(b_ij)

        # reshape A and b
        A = np.array(A).reshape(-1, self.input_dim)
        b = np.array(b).reshape(-1, 1)

        return A, b
    
    def check_barrier(self):
        
        for obstacle in self.obstacles:
            # Control Barrier Function
            # h_{ij}(x) = ||x_i - x_{obs,j}||^2 - d^2

            delta_p = self.state - obstacle
            delta_p_norm = np.linalg.norm(delta_p)

            if delta_p_norm <= self.distance*1.2:
                print("[WARN] Distance NOT Safe")
                return False

        return True
    
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
            c_bar = self.max_velocity*np.ones((n, 1))
  
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

            if np.any(np.abs(safe_input) >= np.array(self.max_velocity)) or np.any(np.isnan(safe_input)):
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