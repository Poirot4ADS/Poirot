import os
import json

def modify_component_conf(component):
    with open("config/config.json","r") as f:
        data = json.load(f)
    conf = data["component_conf"]
    target_conf_settings = conf[component]

    for topic, conf_settings in conf.items():
        modify_flag(conf_settings, "false")
    modify_flag(target_conf_settings, "true")

def modify_flag(settings, value):
    with open(settings[0], 'r') as file:
        lines = file.readlines()
    with open(settings[0], 'w') as file:
        for line in lines:
            if line.startswith(settings[1]):
                file.write(settings[1] + ': ' + value + '\n')
            else:
                file.write(line)


# modify_component_conf("/apollo/localization/pose")