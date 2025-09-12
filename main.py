import json
import time
import lgsvl
import threading
from loguru import logger

from toolset import modify, start_writer, violation_detect, planning, calculate_score
from benchmark import test_scenario
from benchmark.NPC_behavior import action

def RCAnalysis(components, level):
    comp_chain = []
    logger.info("===== Start RCAnalysis =====")
   
    with open("data/score/" + scenario + '.json', "r") as f:
        score_data = json.load(f)
    for component in components:
        c_score = score_data[component]
        comp_chain.append([component, c_score])

    while(1):
        global v_flag
        v_flag = False # default not occur violation
        f_comp = [None, -1]
        if level == "module":
            f_comp = comp_chain[-1]
        elif level == "component":

            if len(comp_chain) == 1:
                return comp_chain[0][0]
            
            # choose component
            for comp_info in comp_chain:
                if comp_info[1] >= f_comp[1]:
                    f_comp = comp_info

        
        modify.modify_component_conf(f_comp[0])
        
        writer = gt_writers[f_comp[0]]
        cmd = writer + scenario + '.record.00000' if module != 'localization' else writer

    
        file_path = 'benchmark/scenario/' + scenario + '.json'
        with open(file_path, "r") as f:
            data = json.load(f)
        # init scene
        label = data["label"]
        environment = data["environment"]
        ego_id = data["ego_id"]
        # sim.reset()
        test_scenario.initial_scene(sim, environment, data['lgsvl_map'])

        
        logger.info(f"===== Scenario : {data['scenario_name']} =====")
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
        ego = test_scenario.set_ego(sim, ego_id, ego_start_pos, speed)
        def detect_collision(agent1, agent2, contact):
            global v_flag
            v_flag = True
            logger.error("collision!")
        ego.on_collision(detect_collision)

        # set npcs
        npc_list = []
        for npc_data in npcs_data:
            npc_start_pos = lgsvl.Vector(npc_data["position"]["x"], npc_data["position"]["y"], npc_data["position"]["z"])
            npc_name = npc_data["type"]
            npc = test_scenario.set_npc(sim, npc_name, npc_start_pos)
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
        traffic_lights_info = None
        try:
            traffic_lights_info = data["trafficlight_loc"]
            test_scenario.set_traffic_lights(sim, traffic_lights_info)
        except KeyError:
            logger.info("No signal control")
        
        # set error line info
        error_line_info = None
        try:
            error_line_info = data["error_line_info"]
        except KeyError:
            logger.info("No warning lines")

        # set apollo
        ego_destination = lgsvl.Vector(ego_data["destination"]["x"], ego_data["destination"]["y"], ego_data["destination"]["z"])
        dv = test_scenario.bridgeApollo(sim, ego, ego_destination, data['apollo_map'])

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
        
        time.sleep(5)
        test_time = 0

        
        set_writer = False
        temp_position = None
        stop_unreasonable = 0
        while test_time <= 40:
            sim.run(0.1)
            
            if not set_writer:
                set_writer = True
                result = start_writer.run_docker_command(docker_settings["user"],docker_settings["docker_name"], cmd)
                #time.sleep(1)
                print(result)

            test_time += 0.1
            ego_state = ego.state
            if not violation_detect.close_to_des(ego_state.position, ego_destination, 3.5):
                if violation_detect.close_to_des(temp_position, ego_state.position, 0.15):
                    stop_unreasonable += 1
                    if stop_unreasonable >= 180:
                        logger.error("long time no reason stop!")
                        v_flag = True
                        break
                else:
                    stop_unreasonable = 0
            else: 
                stop_unreasonable = 0

            
            for actionInfo in npc_list:
                npc = actionInfo[0]
                p = threading.Thread(target=violation_detect.judge_violation, args=(sim, [ego_state, ego.bounding_box], [traffic_lights_info, error_line_info], v_flag))
                p.start()
                p.join()
                
                if v_flag: break
            
            
            temp_position = ego_state.position
            
            if v_flag : break

        # dv.disable_module('Recorder')
            
        logger.info("============ Re-Simulation Over =============")
        
        if level == "module":
            return v_flag
        elif level == "component":
            if v_flag:
                comp_chain = comp_chain[comp_chain.index(f_comp)+1:]
            else:
                comp_chain = comp_chain[:comp_chain.index(f_comp)+1]



if __name__ == '__main__':
    
    guilty_module = None
    guilty_component = None
    with open("config/config.json", "r") as f:
        config = json.load(f)
    scenario = config["scenario"]
    pool = config["modules"]
    gt_writers = config["custom_gt_writer"]
    component_conf = config["component_conf"]
    docker_settings = config["docker_settings"]

    
    calculate_score.cal_score(scenario)
    now_timestamp = time.time()
    
    sim = test_scenario.connect_lgsvl()
    for module, components in pool.items():
        print(module)
        if module != "planning" and module != "control":
            
            if RCAnalysis(components, "module"):
                global v_flag
                print("main : ", v_flag)
                continue
            else:
                guilty_module = module
                guilty_component = RCAnalysis(components, "component")
                break
        
        elif module == "planning":
            if planning.analysis_planning(scenario):
                guilty_module = module
                guilty_component = components
                break
        
        elif module == "control":
            guilty_module = module
            guilty_component = components
            break
    
    logger.info(f"***** {guilty_module} / {guilty_component} *****")
    end_timestamp = time.time()
    print(f"=== time use {end_timestamp - now_timestamp} ===")



