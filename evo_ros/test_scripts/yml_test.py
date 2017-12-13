#!/usr/bin/env python

import yaml
import os
import sys
import re

config_file_name = 'robonodes.yml'
print(__file__)

with open(os.path.dirname(os.path.abspath(__file__)) + '/../config/{}'.format(config_file_name), 'r') as ymlfile:
	cfg = yaml.load(ymlfile)

"""
for section in cfg:
	print(section)
	print(cfg[section])
"""

for worker in cfg['worker_list']:
	print(str(worker))
	print('\t {}'.format(cfg['worker_list'][str(worker)]['ip']))	

#MAVPROXY_CMD_STR = cfg['software_manager']['SCRIPTS']['copter']['MAVPROXY_CMD_STR']
#os.system("xterm -title 'MAVProxy' -hold  -e '{}'&".format(MAVPROXY_CMD_STR))
