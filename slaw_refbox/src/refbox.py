#!/usr/bin/python
from refereeBoxClient import *
from os import getenv
import subprocess

#SERVER = "10.10.16.118" #"192.168.51.117" #"10.10.16.207"
#SERVER = "10.10.16.118" #"192.168.1.53"
SERVER = "192.168.142.1"
PORT = "11111"
TEAM = "swarmlab@work"

ON = True #False
#ON = False

BNT_PLAN_FILE = getenv("HOME") + "/ros/swarmlabatwork/slaw_bnt/config/plan.yaml"
BMT_PLAN_FILE = getenv("HOME") + "/ros/swarmlabatwork/slaw_bmt/config/plan.yaml"  
BTT_PLAN_FILE = getenv("HOME") + "/ros/swarmlabatwork/slaw_btt/config/plan.yaml"
PPT_PLAN_FILE = getenv("HOME") + "/ros/swarmlabatwork/slaw_ppt/config/plan.yaml"
CBT_PLAN_FILE = getenv("HOME") + "/ros/swarmlabatwork/slaw_cbt/config/plan.yaml"

def parse_msg(msg):
	test = msg[0:msg.index("<")]
	goals = msg[msg.index("<")+1:-1] # remove closing bracket

	print "\nTEST: %s\n"%test
	
	if test == "BNT":
		parse_bnt(goals)

	if test == "BMT":
		parse_bmt(goals)

	if test == "BTT":
		parse_btt(goals)

	if test == "PPT":
		parse_ppt(goals)

	if test == "CBT":
		parse_cbt(goals)

def parse_ppt(goals):
	source_place = goals.split(",")[0]
	destination_place = goals[goals.find(")") + 2:]
	obj_list = goals[goals.find("(") + 1:goals.find(")")].split(",")

	# source location
	out = "locations:\n"
	out += "  - {name: %s}\n"%source_place
	out += "\n"

	# Objects at location
	out += "%s:\n"%source_place
	out += "  objects:\n"
	for obj in obj_list:
		out += "    - {name: %s}\n"%obj
	out += "\n"

	# destination location
	for obj in obj_list:
		out += "%s:\n"%obj
		out += "  place_location: %s\n"%destination_place
		out += "\n"

	print out

	print "\nsaving plan to file (%s)...\n"%PPT_PLAN_FILE

	f = open(PPT_PLAN_FILE, "w")
	f.write(out)
	f.close()

	if ON:
	# clear params
		# start launch file
		process = subprocess.Popen(["roslaunch", "slaw_ppt", "ppt.launch"])
		process.wait()
	else:
		process = subprocess.Popen(["ping", "127.0.0.1"])
		process.wait()

def parse_cbt(goals):
	source_place = goals
	
	# source location
	out = "locations:\n"
	out += "  - {name: %s}\n"%source_place
	out += "\n"
	
	print out

	print "\nsaving plan to file (%s)...\n"%CBT_PLAN_FILE

	f = open(CBT_PLAN_FILE, "w")
	f.write(out)
	f.close()

	if ON:
		# start launch file
		process = subprocess.Popen(["roslaunch", "slaw_cbt", "cbt.launch"])
		process.wait()
	else:
		process = subprocess.Popen(["ping", "127.0.0.1"])
		process.wait()


def parse_bnt(goals):
	global ON
	goal_list = goals.split("(") # split at "("
	del(goal_list[0]) # delete empty first item
	for i,g in enumerate(goal_list):
		goal_list[i] = g[0:-1] # delete ")"
	goal_list.append("D0,W,3") # end location
		
	out = "locations:\n"
	for g in goal_list:
		label, direction, sleep = tuple(g.split(","))
		out += "  - {name: %s, dir: %s, sleep: %s}\n"%(label, direction, sleep)
		
	print out

	print "\nsaving plan to file (%s)...\n"%BNT_PLAN_FILE

	f = open(BNT_PLAN_FILE, "w")
	f.write(out)
	f.close()

       	if ON:
		# start launch file
		process = subprocess.Popen(["roslaunch", "slaw_bnt", "bnt.launch"])
		process.wait()
	else:
		process = subprocess.Popen(["ping", "127.0.0.1"])
		process.wait()

	
def parse_bmt(goals):
	print goals
	
	goal_list = goals.split(",")

	initial = goal_list[0]
	source_place = goal_list[1]
	destination_place = goal_list[2]
	configuration = goal_list[3][0:goal_list[3].find("(")]
	object_list = goals[goals.find("(") + 1:goals.find(")")].split(",")
	final_place = goals[goals.find(")")+2:len(goals)]

	print "_____________________________"
	print "initial: %s"%initial
	print "source_place: %s"%source_place
	print "destination_place: %s"%destination_place
	print "configuration: %s"%configuration
	print "objects:"%object_list
	for o in object_list:
		print "\t%s"%o
	print "final_place: %s"%final_place
	print "_____________________________"

	# Locations
	out = "locations:\n"
	out += "  - {name: %s}\n"%source_place
	out += "  - {name: %s}\n"%final_place
	out += "\n"

	# Objects at locations
	out += "%s:\n"%source_place
	out += "  objects:\n"
	for o in object_list:
		out += "    - {name: %s}\n"%o
	out += "\n"

	# Object destionations
	for o in object_list:
		out += "%s:\n"%o
		out += "  place_location: %s\n"%destination_place
		out += "\n"
	
	print out

	print "\nsaving plan to file (%s)...\n"%BMT_PLAN_FILE

	f = open(BMT_PLAN_FILE, "w")
	f.write(out)
	f.close()

	if ON:
		process = subprocess.Popen(["roslaunch", "slaw_bmt", "bmt.launch"])
		process.wait()
	else:
       		process = subprocess.Popen(["ping", "127.0.0.1"])
		process.wait()

def parse_btt(msg):
	print msg
	msg = msg.strip()

	initial_msg,goal_msg = msg.split(';')
	initial_msg = initial_msg.strip()
	goal_msg = goal_msg.strip()
	
	initial_msg = initial_msg[17:-1]
	goal_msg = goal_msg[14:-1]

	print"\n"
	print initial_msg
	print goal_msg
	print"\n"

	initial_list = initial_msg.split('<')
	del(initial_list[0])
	for i,g in enumerate(initial_list):
		initial_list[i] = g[0:-1]

	goal_list = goal_msg.split('<')
	del(goal_list[0])
	for i,g in enumerate(goal_list):
		goal_list[i] = g[0:-1]

	# Initial states
	no_of_states = 0
	initial_states = {}
	for g in initial_list:
		state_name = g[0:g.index(',')]
		configuration = g[g.index(',') + 1:g.find("(")]
		obj = g[g.find("(") + 1:g.find(")")]
		if state_name in initial_states.keys():
			state = initial_states[state_name]	
			state.append((obj, configuration))
		else :
			obj_list = []
			obj_list.append((obj, configuration))
			initial_states[state_name] = obj_list
			no_of_states += 1

	# Goal states
	goal_states = []
	for g in goal_list:
		name = g[0:g.index(',')]
		configuration = g[g.index(',') + 1:g.find("(")]
		obj = g[g.find("(") + 1:g.find(")")]
		goal_states.append((name, configuration, obj))

	destination = goal_states[0][0]

	# sort initial_states
	items_sorted = []
	item_to_sort = ""
	while (len(items_sorted) < no_of_states):
		no_of_items = 0
		for (name, obj_list) in initial_states.items():
			print "Name: %s"%name
			for obj in obj_list:
				print "Obj: %s"%obj[0]
				no_of_objects = len(obj[0].split(","))
				print "No of objects %s"%no_of_objects
			if not name in items_sorted: 
				if no_of_objects > no_of_items:
					item_to_sort = name
					no_of_items = no_of_objects
		items_sorted.append(item_to_sort)
		print "Adding %s"%item_to_sort
		print "No of objects when adding %s"%no_of_items
		

	out = "locations:\n"
	for item in items_sorted:
		out += "  - {name: %s}\n"%item

	out += "\n"
	
	# Objects at locations
	for (name, object_list) in initial_states.items():
		out += "%s:\n"%name
		out += "  objects:\n"
		for obj in object_list:
			obj_name = obj[0]
			objects = obj_name.split(",")
			for object_at_location in objects:
				out += "    - {name: %s}\n"%(object_at_location)	
		out += "\n"

	# Object destinations
	for (name, configuration, obj_list) in goal_states:
		objects = obj_list.split(",")
		for obj in objects:
			out += "%s:\n"%obj
			out += "  place_location: %s\n"%name
			out += "\n"

	print "\n%s"%out

	print "\nsaving plan to file (%s)...\n"%BTT_PLAN_FILE

	f = open(BTT_PLAN_FILE, "w")
	f.write(out)
	f.close()

	if ON:
		# start launch file
		process = subprocess.Popen(["roslaunch", "slaw_btt", "btt.launch"])
		process.wait()
	else:
		process = subprocess.Popen(["ping", "127.0.0.1"])
		process.wait()

	
if __name__ == "__main__":
	msg = obtainTaskSpecFromServer(SERVER, PORT, TEAM)
	print msg
	parse_msg(msg)
