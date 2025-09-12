import math
import pickle
from environs import Env
from loguru import logger
import lgsvl
import json
import os
import time
from benchmark.NPC_behavior import action


modules = [
        'Localization',
        'Transform',
        'Routing',
        'Prediction',
        'Planning',
        'Control',
        'Perception',
        'Recorder'
    ]

def raycast_to_ground(sim, position):
    start_height = 100
    start_point = lgsvl.Vector(
        position.x, position.y + start_height, position.z)
    hit = sim.raycast(start_point, lgsvl.Vector(0, -1, 0), layer_mask=1 << 0)
    if hit:
        return hit.point
    else:
        return position

def connect_lgsvl():
    SIMULATOR_HOST = os.environ.get("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host)
    SIMULATOR_PORT = os.environ.get("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port)
    logger.info("Connecting to the Simulator")
    # Connects to the simulator instance at the ip defined by LGSVL__SIMULATOR_HOST, default is localhost or 127.0.0.1

    sim = lgsvl.Simulator(SIMULATOR_HOST, SIMULATOR_PORT)

    logger.info("Connect to simulator successfully, Version =", sim.version)
    return sim

def initial_scene(sim, environment, map_name):
    if sim.current_scene == map_name:
        sim.reset()
    else:
        sim.load(map_name)
    logger.info("Load map successfully!")
    
    sim.weather = lgsvl.WeatherState(
        rain = environment["rain"],
        fog = environment["fog"],
        wetness = environment["wetness"],
        cloudiness = environment["cloudiness"]
    )
    sim.set_time_of_day(environment["time"])
    return sim

def bridgeApollo(sim, ego, ego_destination, map_name):
    dv = None
    BRIDGE_HOST = os.environ.get("BRIDGE_HOST", "127.0.0.1")
    BRIDGE_PORT = int(os.environ.get("BRIDGE_PORT", 9090))
    ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT) #address, port
    
    times = 0
    success = False
    
    while times < 3:
        try:
            dv = lgsvl.dreamview.Connection(sim, ego, os.environ.get("BRIDGE_HOST", "127.0.0.1"))
            dv.disable_module('Recorder')
            dv.set_hd_map(map_name)
            dv.set_vehicle('Lincoln2017MKZ_LGSVL')
            dv.setup_apollo(ego_destination.x, ego_destination.z, modules, default_timeout=30)
            success = True
            break
        except Exception as e:
            logger.error(e)
            logger.warning('Fail to connect with apollo, try again!')
            module_status = dv.get_module_status()
            for module, status in module_status.items():
                print(module, status)
                if (not status) and (module in modules):
                    logger.warning('$$Simulator$$ Module is closed: ' + module + ' ==> restart')
                    dv.enable_module(module)
            times += 0.1
    if not success:
        raise RuntimeError('Fail to connect with apollo')
 
    dv.set_destination(ego_destination.x, ego_destination.z)
    # logger.info(' --- Set ego_destination: ' + str(ego_destination.x) + ',' + str(ego_destination.z))
    
    delay_t = 3
    time.sleep(delay_t)
    return dv
    
def set_ego(sim, id, start_pos, speed):
    ego_state = lgsvl.AgentState()
    start_pos = raycast_to_ground(sim, start_pos)
    ego_state.transform.position = start_pos
    ego_state.transform = sim.map_point_on_lane(start_pos)
    right = lgsvl.utils.transform_to_right(ego_state.transform)
    forward = lgsvl.utils.transform_to_forward(ego_state.transform)
    # ego_state.transform.position += 7 * right
    # ego_state.transform.position += 60 * forward
    #print(ego_state.transform.position)
    ego_state.velocity = speed * forward
    #ego = sim.add_agent(os.environ.get(
    #     "LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis), lgsvl.AgentType.EGO, ego_state)
    ego = sim.add_agent(id, lgsvl.AgentType.EGO, ego_state)
    #print(ego.state.position)
    logger.info(f"set ego at start pos :{start_pos}.")
    return ego

def set_npc(sim, name, start_pos):
    npc_state = lgsvl.AgentState()
    start_pos = raycast_to_ground(sim, start_pos)
    npc_state.transform.position = start_pos
    npc_state.transform = sim.map_point_on_lane(start_pos)
    npc = sim.add_agent(name, lgsvl.AgentType.NPC, npc_state)
    logger.info(f"Set {name} at start_pos: {start_pos}.")
    return npc

def set_traffic_lights(sim, trafficlight_loc):
    for status, locations in trafficlight_loc.items():
        for loc in locations:
            signal = sim.get_controllable(lgsvl.Vector(loc[0], loc[1], loc[2]), 'signal')
            signal.control(status)
            logger.info(f"Set {loc} signal {status}")

def save_image(ego, path):
    sensors = ego.get_sensors()
    for s in sensors:
        print(s.name)
        if s.name == "Lidar":
            lidar_path = os.path.join(path, 'lidar.pcd')
            s.save(lidar_path)
        if s.name == "Main Camera":
            # Camera and LIDAR sensors can save data to the specified file path
            camera_path = os.path.join(path, 'main-camera.png')
            s.save(camera_path, compression=0)


if __name__ == '__main__':
    # load scenario json
    # scenario_path = '/mnt/sda/RCAnalysis/benchmark/scenario'
    # for filename in os.listdir(scenario_path):
    #     if filename.endswith('.json'):
    #         file_path = os.path.join(scenario_path, filename)
            fi_name = 'rtk_main'
            file_path = 'benchmark/scenario/' + fi_name + '.json'
            with open(file_path, "r") as f:
                data = json.load(f)
            # init scene
            label = data["label"]
            environment = data["environment"]
            ego_id = data["ego_id"]
            sim = connect_lgsvl()
            initial_scene(sim, environment, data['lgsvl_map'])

            
            logger.info(f"== Scenario : {data['scenario_name']} ==")
            ego_data = data["agents"]["ego"]
            npcs_data = data["agents"]["npcs"]
            # set ego
            ego_start_pos = lgsvl.Vector(ego_data["position"]["x"], ego_data["position"]["y"], ego_data["position"]["z"])
            try:
                speed = ego_data["speed"]
                logger.info(f"ego has initial speed {speed}.")
            except KeyError as e:
                speed = 0
                logger.info("ego has no initial speed.")
            ego = set_ego(sim, ego_id, ego_start_pos, speed)

            # set npcs
            npc_list = []
            for npc_data in npcs_data:
                npc_start_pos = lgsvl.Vector(npc_data["position"]["x"], npc_data["position"]["y"], npc_data["position"]["z"])
                npc_name = npc_data["type"]
                npc = set_npc(sim, npc_name, npc_start_pos)
                waypoints = []
                waypoints = action.genWaypoints(sim, npc, npc_data["speed"], npc_data["action"], npc_data["destination"], label)
                npc_list.append((npc, waypoints))
            
            # set pedestrians
            pedestrians = []
            try:
                p_list = data["agents"]["pedestrian"]
                for p in p_list:
                    p_position = lgsvl.Vector(p["position"]["x"], p["position"]["y"], p["position"]["z"])
                    p_des = lgsvl.Vector(p["destination"]["x"], p["destination"]["y"], p["destination"]["z"])
                    p_state = lgsvl.AgentState()
                    p_state.transform = lgsvl.Transform(p_position)
                    pedestrian = sim.add_agent(p["type"], lgsvl.AgentType.PEDESTRIAN, p_state)
                    pedestrians.append((pedestrian, p_des))
            except KeyError:
                logger.info("No pedestrians")
            
            # set traffic cone
            try:
                cones = []
                c_list = data["cone"]
                for c in c_list:
                    cone_state = lgsvl.ObjectState()
                    cone_state.transform = lgsvl.Transform(lgsvl.Vector(c["x"], c["y"], c["z"]), lgsvl.Vector(0, 0, 0))
                    cone = sim.controllable_add("TrafficCone", cone_state)
                    cones.append(cone)
            except KeyError as e:
                logger.info("No traffic cones")

            # set traffic     
            try:
                set_traffic_lights(sim, data["trafficlight_loc"])
            except KeyError:
                logger.info("No signal control")

            # set apollo
            ego_destination = lgsvl.Vector(ego_data["destination"]["x"], ego_data["destination"]["y"], ego_data["destination"]["z"])
            dv = bridgeApollo(sim, ego, ego_destination, data['apollo_map'])

            # set action
            for actionInfo in npc_list:
                npc = actionInfo[0]
                waypoints = actionInfo[1]
                if waypoints == []:
                    continue
                else:
                    npc.follow(waypoints, loop=False)
                    logger.info(f"{npc.name} starts moving")
            for act in pedestrians:
                ped = act[0]
                des = act[1]
                ped.walk_randomly(True)
            
            # print(sim.get_controllable(ego.state.position, "signal"))
            # 100hz = /0.01s
            time.sleep(5)
            test_time = 0

            # SIM and GT record
            gt_record = {"ego":[]}
            for info in npc_list:
                gt_record.setdefault(info[0].name,[])

            while test_time <= 40:
                sim.run(0.1)

                # p = '/mnt/sda/RCAnalysis/benchmark/record/I2R/' + fi_name
                # save_image(ego, p)
                logger.info(ego.state.position)
                gps_data = sim.map_to_gps(ego.state.transform)
                gt_record["ego"].append([ego.state, ego.bounding_box, gps_data])
                for info in npc_list:
                    gt_record[info[0].name].append([info[0].state, info[0].bounding_box, sim.map_to_gps(info[0].state.transform)])

                test_time += 0.1
            dv.disable_module('Recorder')

            # Save record
            save_path = ' ' + fi_name + '.obj'
            with open(save_path, 'wb') as f:
                pickle.dump(gt_record, f)