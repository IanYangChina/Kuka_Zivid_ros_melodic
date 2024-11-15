import numpy as np
import os
import taichi as ti
from time import time
from doma.envs.sys_id_env import make_env
from doma.engine.utils.misc import set_parameters

DTYPE_NP = np.float32
DTYPE_TI = ti.f32
cuda_GB = 5
script_path = os.path.dirname(os.path.realpath(__file__))

loss_cfg = {
    'planning': True,
    'target_ind': 0,
    'height_map_res': 32,
    'height_map_size': 0.11,
}

data_cfg = {
    'data_path': os.path.join(script_path, '..', '..', '..',
                              'test', 'deformable_objects', 'pcd_to_mesh'),
    'data_ind': str(0),
    'target_hm_path' : os.path.join(script_path, '..', '..', '..',
                                    'test', 'deformable_objects', 'data-planning-targets')
}

env_cfg = {
    'p_density': 2e7,
    'horizon': 500,
    'dt_global': 0.01,
    'n_substeps': 50,
    'material_id': 2,
    'agent_name': 'cylinder',
    'agent_init_euler': (0, 0, 0),
}

cam_cfg = {
    'pos': (0.40, 0.1, 0.1),
    'lookat': (0.25, 0.25, 0.05),
    'fov': 30,
    'lights': [{'pos': (0.5, 0.25, 0.2), 'color': (0.6, 0.6, 0.6)},
               {'pos': (0.5, 0.5, 1.0), 'color': (0.6, 0.6, 0.6)},
               {'pos': (0.5, 0.0, 1.0), 'color': (0.8, 0.8, 0.8)}],
    'particle_radius': 0.002,
    'res': (640, 640),
    'euler': (135, 0, 180),
    'focal_length': 0.01
}

params = np.load(os.path.join(script_path, 'MPM_planning', 'DPSI_found_params.npy')).flatten()
E = params[0]
nu = params[1]
yield_stress = params[2]
rho = params[3]
mf = params[4]
gf = np.array([1.6], dtype=DTYPE_NP)


def reset_ti_and_env():
    ti.reset()
    ti.init(arch=ti.cuda, default_fp=DTYPE_TI, default_ip=ti.i32, fast_math=True, random_seed=0,
            debug=False, check_out_of_bound=False, device_memory_GB=cuda_GB,
            advanced_optimization=True)
    env, mpm_env, _ = make_env(data_cfg, env_cfg, loss_cfg, cam_cfg)
    set_parameters(mpm_env, env_cfg['material_id'],
                   e=E.copy(), nu=nu.copy(), yield_stress=yield_stress.copy(), rho=rho.copy(),
                   ground_friction=gf.copy(),
                   manipulator_friction=mf.copy())
    return env, mpm_env


def forward(mpm_env, init_state, init_agent_pos, trajectory, render=False):
    # Forward
    mpm_env.set_state(init_state['state'], grad_enabled=False)
    init_agent_p = np.append(init_agent_pos, mpm_env.agent.effectors[0].init_rot)
    mpm_env.agent.effectors[0].set_state(0, init_agent_p)
    mpm_env.agent.effectors[0].mesh.update_color((0.2, 0.2, 0.2, 1.0))
    if render:
        mpm_env.render("human")
    for i in range(trajectory.shape[0]):
        action = trajectory[i]
        mpm_env.step(action)
        if render:
            mpm_env.render("human")

    loss_info = mpm_env.get_final_loss()
    return loss_info


def simulate(action, data_ind, target_ind):
    data_cfg['data_ind'] = data_ind
    loss_cfg['target_ind'] = target_ind
    ti.reset()
    ti.init(arch=ti.cuda, default_fp=DTYPE_TI, default_ip=ti.i32, fast_math=True, random_seed=0,
            debug=False, check_out_of_bound=False, device_memory_GB=cuda_GB,
            advanced_optimization=True)
    env, mpm_env, new_init_state = make_env(data_cfg, env_cfg, loss_cfg, cam_cfg)
    init_agent_pos = np.asarray(mpm_env.agent.effectors[0].init_pos)
    agent_init_pos = init_agent_pos + np.array([action[1] * 0.02, 0, 0])
    if action[0] == 0:
        trajectory = np.zeros(shape=(2, 6))
    else:
        trajectory = np.load(os.path.join(script_path, 'MPM_planning',
                                          f'tr_poking-shifting_{action[0]}_v_dt_0.01.npy'))
    forward(mpm_env, new_init_state, agent_init_pos, trajectory, render=True)
    ti.reset()


def find_best_action_n(data_ind, target_ind, n=2):
    best_action, best_loss, finished_state, resultant_heightmap_z_max_ = find_best_action(data_ind, target_ind)
    actions = [best_action]
    for i in range(n-1):
        best_action, best_loss, finished_state, resultant_heightmap_z_max_ = \
            find_best_action(data_ind, target_ind, finished_state, resultant_heightmap_z_max_)
        actions.append(best_action)
    return actions


def find_best_action(data_ind, target_ind, init_state=None, resultant_heightmap_z_max=None):
    data_cfg['data_ind'] = data_ind
    loss_cfg['target_ind'] = target_ind
    ti.reset()
    ti.init(arch=ti.cuda, default_fp=DTYPE_TI, default_ip=ti.i32, fast_math=True, random_seed=0,
            debug=False, check_out_of_bound=False, device_memory_GB=cuda_GB)
    env, mpm_env, new_init_state = make_env(data_cfg, env_cfg, loss_cfg, cam_cfg)
    if init_state is not None:
        new_init_state = init_state
    init_agent_pos = np.asarray(mpm_env.agent.effectors[0].init_pos)

    best_action = (0, 0, 0)
    best_loss = np.inf
    finished_state = None
    resultant_heightmap_z_max_ = resultant_heightmap_z_max
    for action in [1, 2]:
        trajectory = np.load(os.path.join(script_path, 'MPM_planning',
                                          f'tr_poking-shifting_{action}_v_dt_0.01.npy'))
        for x in range(-1, 2):
            t0 = time()
            env, mpm_env = reset_ti_and_env()
            agent_init_pos = init_agent_pos + np.array([x * 0.02, 0, 0])
            if resultant_heightmap_z_max is not None:
                agent_init_pos[2] = resultant_heightmap_z_max / 1000
            print(f"Trying action {action}, location offset {x * 0.02} in x axix")
            loss_info = forward(mpm_env, new_init_state, agent_init_pos, trajectory)
            print(f"Loss: {loss_info['height_map_loss_pcd']}")
            if loss_info['height_map_loss_pcd'] < best_loss:
                best_loss = loss_info['height_map_loss_pcd']
                best_action = (action, x, 0, np.array([resultant_heightmap_z_max]).tolist()[0])
                resultant_heightmap_z_max_ = np.max(loss_info['final_height_map'])
                finished_state = mpm_env.get_state()
                if resultant_heightmap_z_max_ < 0.02:
                    resultant_heightmap_z_max_ = 0.02
            print(f"Time taken: {time() - t0}")

    t0 = time()
    env, mpm_env = reset_ti_and_env()
    agent_init_pos = init_agent_pos
    print(f"Trying zero action")
    # input("Press Enter to continue...")
    loss_info = forward(mpm_env, new_init_state, agent_init_pos, np.zeros(shape=(2, 6)))
    print(f"Loss: {loss_info['height_map_loss_pcd']}")
    if loss_info['height_map_loss_pcd'] < best_loss:
        best_loss = loss_info['height_map_loss_pcd']
        best_action = (0, 0, 0, np.array([resultant_heightmap_z_max]).tolist()[0])
        resultant_heightmap_z_max_ = np.max(loss_info['final_height_map'])
        finished_state = mpm_env.get_state()
        if resultant_heightmap_z_max_ < 0.02:
            resultant_heightmap_z_max_ = 0.02
    print(f"Time taken: {time() - t0}")
    ti.reset()

    np.save(os.path.join(data_cfg['data_path'], f'sim_hm_{data_ind}.npy'),
            loss_info['final_height_map'])
    return best_action, best_loss, finished_state, resultant_heightmap_z_max_
