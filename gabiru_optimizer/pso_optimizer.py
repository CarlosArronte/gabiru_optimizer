import numpy as np
from scipy.interpolate import interp1d
import pyswarms as ps

def interpolate_waypoints(waypoints, ref_length=1000):
    N = waypoints.shape[0]
    x_interp_func = interp1d(np.arange(N), waypoints[:,0], kind='linear')
    y_interp_func = interp1d(np.arange(N), waypoints[:,1], kind='linear')
    x_new = x_interp_func(np.linspace(0, N-1, ref_length))
    y_new = y_interp_func(np.linspace(0, N-1, ref_length))
    return np.vstack((x_new, y_new)).T

def simulate_pure_pursuit(params, waypoints, ref_xy):
    lookahead, velocity, max_ang_vel = params

    pose = np.array([waypoints[0,0], waypoints[0,1], 0.0])  # x, y, theta
    poses = [pose.copy()]

    dt = 0.05
    max_time = 60
    goal_radius = 1.0
    idx_goal = waypoints.shape[0] - 1

    def dist(a, b):
        return np.linalg.norm(a[:2] - b)

    time = 0
    while dist(pose, waypoints[idx_goal]) > goal_radius and time < max_time:
        dists = np.linalg.norm(waypoints - pose[:2], axis=1)
        targets = np.where(dists > lookahead)[0]
        goal_point = waypoints[targets[0]] if len(targets) > 0 else waypoints[-1]

        alpha = np.arctan2(goal_point[1] - pose[1], goal_point[0] - pose[0]) - pose[2]
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

        omega = 2 * velocity * np.sin(alpha) / lookahead
        omega = np.clip(omega, -max_ang_vel, max_ang_vel)
        v = velocity

        pose[0] += v * np.cos(pose[2]) * dt
        pose[1] += v * np.sin(pose[2]) * dt
        pose[2] += omega * dt
        pose[2] = (pose[2] + np.pi) % (2 * np.pi) - np.pi

        poses.append(pose.copy())
        time += dt

    traj = np.array(poses)[:,:2]
    min_errors = [np.min(np.linalg.norm(ref_xy - p, axis=1)) for p in traj]
    rmse = np.sqrt(np.mean(np.square(min_errors)))
    if dist(pose, waypoints[idx_goal]) > goal_radius:
        rmse += 50
    return rmse

# Función que pyswarms espera: evalúa *varias* partículas al mismo tiempo
def objective_function(x, waypoints, ref_xy):
    # x es array (n_particles, n_params)
    n_particles = x.shape[0]
    j = np.zeros(n_particles)
    for i in range(n_particles):
        j[i] = simulate_pure_pursuit(x[i], waypoints, ref_xy)
    return j

def get_optimizer_params(tipo_segmento):
    """
    Retorna los límites (bounds) y las opciones del optimizador PSO
    según el tipo de segmento (recta, curva_suave, curva_fechada).
    """

    if tipo_segmento == 'recta':
        lb = [1.0, 6.0, 1.0]
        ub = [2.5, 10.0, 3.0]
    elif tipo_segmento == 'curva_suave':
        lb = [0.8, 4.0, 2.0]
        ub = [2.0, 8.0, 5.0]
    elif tipo_segmento == 'curva_fechada':
        lb = [0.3, 2.0, 4.0]
        ub = [1.5, 6.0, 10.0]
    else:
        # Defaults si el tipo es desconocido
        lb = [0.3, 2.0, 2.0]
        ub = [2.5, 10.0, 10.0]

    bounds = (np.array(lb), np.array(ub))

    options = {
        'c1': 0.5,  # coeficiente cognitivo
        'c2': 0.3,  # coeficiente social
        'w': 0.9    # inercia
    }

    return bounds, options

def optimize_segment(tipo_segmento,waypoints):
    ref_xy = interpolate_waypoints(waypoints)
    bounds, options = get_optimizer_params(tipo_segmento)   
    

    optimizer = ps.single.GlobalBestPSO(n_particles=30, dimensions=3, options=options, bounds=bounds)
    best_cost, best_pos = optimizer.optimize(objective_function, iters=40, waypoints=waypoints, ref_xy=ref_xy)

    print(f'Mejor combinación:\nLookahead = {best_pos[0]:.2f}\nVelocidad = {best_pos[1]:.2f}\nMax Angular Velocity = {best_pos[2]:.2f}\nRMSE = {best_cost:.3f}')
    return best_pos, best_cost





if __name__ == "__main__":
    waypoints = np.array([
        [23.1, 33.0],
        [25.05, 30.0],
        [29.77, 24.61],
        [36.9, 21.91],
        [37.95, 19.28],
        [34.35, 15.91],
        [28.35, 13.96],
        [18.98, 9.76],
        [14.1, 6.39],
        [11.85, 0.46]
    ])
    optimize_pp_params(waypoints)
