"""
Created on Mon Oct 22 14:32:39 2018
@author: Dr. Pengfei (Taylor) Li wrote this basic first-order dynamic network loading model in Python for MS State University CEE 8143: Traffic Simulation and Management
Users are welcome to modify and then share it with all the transportation community. 

The example network can be visualized with an open-source GUI, NEXTA (enclosed), developed Dr. X Zhou at Arizona State University 

The code has been tested in Spyder, a Pyhon IDE environment

This code is open-source under GNU general public License (GPL)

Author Contact: pflee2002@gmail.com
"""
import math
import csv
#global variables:
_MAX_LABEL_COST=99999 # initial label cost for label correcting algorithm
g_point_queue_flag=1; # this is reserved for the future as this code only considers the point queue
g_node_list=[] # to contain all the nodes
g_link_list=[] # to contain all the links
g_outbound_node_size=[];
g_outbound_link_no=[];
g_vehicle_link_sequence=[];
g_vehicle_node_sequence=[];
g_vehicle_time_sequence=[];
g_vehicle_current_link_idx=[];
g_vehicle_va_time=[]; # to record the times for each vehicle to arrive the nodes on its path (the A curve, arrival cumulative vehicle counts)
g_vehicle_vt_time=[];# to record the virttimes for each vehicle to arrive the nodes on its path (the V curve, virtial arrival vehicle counts)
g_vehicle_vd_time=[];# # to record the times for each vehicle to depart the nodes on its path
g_sim_vehicle_finish_time=[]; # to record when this vehicle read the destination
g_sim_vehicle_complete_flag=[];  # to record if this vehicle finish its path by the end of traffic simulation
g_seed=221; # the initial random seed for LGC generator
g_link_travel_time=[]; # to link free flow travel times
g_link_from_node=[]; # the entering node of a link: i of (i,j)
g_link_to_node=[]; # toe departing node
g_link_no_of_lanes=[]; # the number of lanes in each link
g_link_length=[]; # link length
g_link_storage_capacity=[]; # the max number of vehicles that can stay on that link (jam density * link length)
LinkOutFlowCapacity=[]; # the max number of vehicles able to leave a link at each time step
LinkInFlowCapacity=[];# the max number of vehicles able to enter a link at each time step
LinkStorageCapacity=[]; 
g_link_queue=[]; # a FIFO data structure to record vehcles' entering and leaving, one for each link 
g_link_temp_queue=[]; # a temp FIFO data strcture
g_from_nodes_to_link = dict();# a map to identify a link from its entering node and leaving node
g_node_x=[]; # x coordiate for nodes
g_node_y=[]; # y coordinate for nodes 
g_number_of_nodes =0; # total number of nodes in the network
g_number_of_links=0; # total number of links in the network
g_number_of_agents=0; # total number of vehicles in the network
g_agent_origin=[]; # the origins of all vehicles
g_agent_destination=[]; # the destinations of all vehicles
g_agent_departure_time=[]; # the departure times of all vehicles
g_agent_list=[]; #all vehicles
g_sim_vehicle_finish_time=[]; # vehicles finish time
g_sim_vehicle_complete_flag=[]; # an indicator if this vehicle has finished before the end of loading
##############
#Loading Horizon
g_time_horizon=99;
############################################
## Read node list
############################################   
with open('input_node.csv','r') as fp:
        lines = fp.readlines()   
        for l in lines[1:]:
                l = l.strip().split(',')  
                try: #redundancy design, failure check (try: except)
                    g_node_list.append(int(l[1]));
                    g_node_x.append(float(l[2]));
                    g_node_y.append(float(l[3]));
                    g_number_of_nodes=g_number_of_nodes+1;
                except:
                        print('Your input_node format is incorrect, please check!')
        print('nodes_number:{}'.format(g_number_of_nodes))      
############################################
## Read link list
############################################
with open('input_link.csv','r') as fl:
        linel = fl.readlines()             
        for l in linel[1:]:
                l = l.strip().split(',')
                try:
                     g_number_of_links=g_number_of_links+1;
                     g_link_from_node.append(int(l[1]));
                     g_link_to_node.append(int(l[2]));
                     g_link_travel_time.append(int(l[6]));
                     g_link_no_of_lanes.append(int(l[4]));
                     g_link_length.append(float(l[3]));  
                     LinkOutFlowCapacity.append([]);
                     LinkInFlowCapacity.append([]);
                     LinkStorageCapacity.append([]);
                     link_storage=int(g_link_length[g_number_of_links-1]/14.0);
                     g_link_storage_capacity.append(link_storage);
                     link_capacity=1800.0/3600.0*g_link_no_of_lanes[g_number_of_links-1];#by default, the saturation rate is set as 1,800 vehicles per hour per lane
                     g_link_storage_capacity.append(link_storage);
                     for t in range(0,g_time_horizon):
                        LinkOutFlowCapacity[g_number_of_links-1].append(link_capacity);
                        LinkInFlowCapacity[g_number_of_links-1].append(link_capacity);
                        LinkStorageCapacity[g_number_of_links-1].append(link_storage);
                     key=l[1]+"_"+l[2];
                     g_from_nodes_to_link[key]=g_number_of_links;
                     g_link_queue.append([]);
                     g_link_temp_queue.append([]);
                     current_size=len(g_outbound_node_size);
                     if(current_size<int(l[1])): # This is a new from node and a new memory should be allocated
                        outbound_node_size=[];
                        outbound_link_list=[];
                        g_outbound_node_size.append(outbound_node_size);
                        g_outbound_link_no.append(outbound_link_list);
                        size=len(g_outbound_node_size);
                        g_outbound_node_size[size-1]=1;
                        g_outbound_link_no[size-1].append(g_number_of_links);
                     else:
                        idx=int(l[1]);
                        g_outbound_node_size[idx-1]=g_outbound_node_size[idx-1]+1;
                        g_outbound_link_no[idx-1].append(g_number_of_links);                      
                except:
                        print('the input link file does not have the right format');
        
############################################
## Read agent list
############################################        
with open('input_agent.csv','r') as fa:
        linea = fa.readlines()
        for l in linea[1:]:
                l = l.strip().split(',')
                try:    
                    g_number_of_agents=g_number_of_agents+1;
                    g_agent_origin.append(int(l[1]));
                    g_agent_destination.append(int(l[2]));
                    g_agent_departure_time.append(int(l[3]));
                    g_agent_list.append(g_number_of_agents);
                    g_sim_vehicle_finish_time.append(0);
                    g_sim_vehicle_complete_flag.append(0); 
                    g_vehicle_current_link_idx.append(0);
                except:
                        print('the input agent file does not have the right format')               
#######################################################################
## Find shortest path for each vehicle with label correcting algorithm
## Users can also load any existing agent paths for loading                        
#######################################################################       
v=1;
while(v<=g_number_of_agents):        
    g_vehicle_node_sequence.append([]) 
    g_vehicle_time_sequence.append([])     
    g_vehicle_link_sequence.append([]) 
    g_vehicle_va_time.append([]) 
    g_vehicle_vt_time.append([]) 
    g_vehicle_vd_time.append([]) 
    origin=g_agent_origin[v-1] 
    destination=g_agent_destination[v-1] 
    departure_time=g_agent_departure_time[v-1] 
    total_travel_time=0.0 
    link_travel_time=[1.0]*(g_number_of_links+1) #allocate memory for link travel time
    total_cost=_MAX_LABEL_COST 
    path_node_sequence=[0]*(g_number_of_nodes+1) 
    path_link_sequence=[0]*(g_number_of_links+1) 
    path_time_sequence=[0]*(g_number_of_nodes+1) 
    node_label_cost=[_MAX_LABEL_COST]*(g_number_of_nodes+1)  # idx starts from 1
    node_predecessor=[-1]*(g_number_of_nodes+1) 
    time_predecessor=[-1]*(g_number_of_nodes+1) 
    for l in range(0,g_number_of_links-1):
        link_travel_time[l]=g_link_travel_time[l] 
    node_label_cost[origin-1]=0 
    open_node_list=[] 
    open_node_list.append(origin) 
    while(len(open_node_list)):
        from_node=open_node_list[0] 
        del open_node_list[0] 
        for idx in range(0, g_outbound_node_size[from_node-1]):
            link_no=g_outbound_link_no[from_node-1][idx] 
            to_node=g_link_to_node[link_no-1] 
            b_node_updated=0 
            temp_label_cost = node_label_cost[from_node-1] + link_travel_time[link_no-1] 
            if temp_label_cost < node_label_cost[to_node-1]:
                node_label_cost[to_node-1] = temp_label_cost 
                node_predecessor[to_node-1] = from_node 
                time_predecessor[to_node-1] = node_label_cost[from_node-1] 
                b_node_updated = 1 
                if g_outbound_node_size[from_node-1] > 0 :
                    if to_node != destination:
                        open_node_list.append(to_node)    
    total_travel_time=node_label_cost[destination-1] 
    reverse_path_node_sequence=[-1]*(g_number_of_nodes+1) 
    reverse_path_time_sequence=[-1]*(g_number_of_nodes+1) 
    node_size=0 
    reverse_path_node_sequence[node_size]=destination 
    reverse_path_time_sequence[node_size]=total_travel_time 
    node_size=node_size+1 
    pred_node=node_predecessor[destination-1] 
    pred_time=time_predecessor[destination-1] 
    while(pred_node!=-1):
       reverse_path_node_sequence[node_size]=pred_node 
       reverse_path_time_sequence[node_size]=pred_time  
       node_size=node_size+1 
       pred_node_record=pred_node 
       pred_time_record=pred_time 
       pred_node=node_predecessor[pred_node_record-1] 
       pred_time=time_predecessor[pred_node_record-1] 
    for n in range(0,node_size):
        g_vehicle_node_sequence[v-1].append(reverse_path_node_sequence[node_size - n - 1]) 
        g_vehicle_time_sequence[v-1].append(reverse_path_time_sequence[node_size - n - 1]+ g_agent_departure_time[v-1]) 
        g_vehicle_va_time[v-1].append(reverse_path_time_sequence[node_size - n - 1]+ g_agent_departure_time[v-1]) 
        g_vehicle_vt_time[v-1].append(reverse_path_time_sequence[node_size - n - 1]+ g_agent_departure_time[v-1]) 
        g_vehicle_vd_time[v-1].append(reverse_path_time_sequence[node_size - n - 1]+ g_agent_departure_time[v-1]) 
        path_node_sequence[n]=reverse_path_node_sequence[node_size - n - 1] 
        path_time_sequence[n] = reverse_path_time_sequence[node_size - n - 1] + g_agent_departure_time[v-1]     
    for l in range(0,node_size-1):
        key=str(path_node_sequence[l])+"_"+str(path_node_sequence[l+1]) 
        link_id = g_from_nodes_to_link[key] 
        g_vehicle_link_sequence[v-1].append(link_id) 
        path_link_sequence[l]=link_id 
    v=v+1           
##############################
##Dynamic Network Loading with Newell car following model and point queue model
##############################
#Variable Initialization
for l in range(0,g_number_of_links):
    del g_link_queue[l][:]; # clear the queues
    for tt in range(0,g_time_horizon):
        LinkOutFlowCapacity[l][tt]=1800.0*g_link_no_of_lanes[l]/3600.0;
        LinkInFlowCapacity[l][tt]=1800.0*g_link_no_of_lanes[l]/3600.0;
        if g_point_queue_flag==1:
            LinkStorageCapacity[l][tt];
        else:
            LinkStorageCapacity[l][tt]=g_link_storage_capacity;
agent_list=g_agent_list[:];
link_queue=g_link_queue[:];
link_temp_queue=g_link_temp_queue[:];
vehicle_va_time=g_vehicle_va_time[:];
vehicle_vt_time=g_vehicle_vt_time[:];
vehicle_vd_time=g_vehicle_vd_time[:];
vehicle_current_link_idx=g_vehicle_current_link_idx[:];
vehicle_time_sequence=g_vehicle_time_sequence[:];
for t in range(0,g_time_horizon): # Simulation begins
    # Step 1: Release vehicles into the network if their departure times reach
    print('t=',t,'\n');
    temp_agent=[];
    for v in agent_list:
        if g_agent_departure_time[v-1]<=t: #this vehicle is ready to depart
            origin_link=g_vehicle_link_sequence[v-1][0];# identify which link this vehicle will enter                
            link_entering_flag=0; # to tell if this vehicle can enter this link?
            if(LinkInFlowCapacity[origin_link-1][t]>=1): # We first check if the link still has inbound capacity for this vehicle to enter at this time
                link_entering_flag=1;
            elif LinkInFlowCapacity[origin_link-1][t]>0 and LinkInFlowCapacity[origin_link-1][t]<1: # we need to toss a dice to determine if this vehicle should cross
                g_seed=(g_seed * 16807.0) % 2147483647.0; #LGC method to generate a random variable
                u = g_seed / 2147483647.0; #random number between 0 to 1
                if (u < LinkInFlowCapacity[origin_link-1][t]):
                    link_entering_flag=1;
                else:
                    link_entering_flag=0;
            if link_entering_flag==1:
                link_queue[origin_link-1].append(v);
                vehicle_va_time[v-1][0]=t;
                vehicle_vt_time[v-1][0]=t+g_link_travel_time[origin_link-1];
                temp_agent.append(v);
                LinkInFlowCapacity[origin_link-1][t]=LinkInFlowCapacity[origin_link-1][t]-1;
            else:
                break;
    for v in temp_agent:
        agent_list.remove(v);
    temp_agent.clear();
    # Step 2: Check if any vehicle has reach the end of its current link and is ready to enter the next link 
    for l in range(1,g_number_of_links):
        while(1):
            if len(link_queue[l-1])==0: # no vehicles on that link and so stop checking
                break;
            front_vehicle=link_queue[l-1][0];
            current_link_idx=vehicle_current_link_idx[front_vehicle-1];
            #if vehicle_run_on_the_way_flag[front_vehicle-1]==1 and vehicle_vt_time[front_vehicle-1][current_link_idx]>=t:# this vehicle has not reached the end of this link. so no worry about it at this step
            if vehicle_vt_time[front_vehicle-1][current_link_idx]>=t:
                break;
            current_link_no=g_vehicle_link_sequence[front_vehicle-1][current_link_idx];
            path_last_link_idx=len(g_vehicle_link_sequence[front_vehicle-1]);
            v_last_link=g_vehicle_link_sequence[front_vehicle-1][path_last_link_idx-1];
            if v_last_link==current_link_no: # this vehicle has reached the end of its path
                next_link_no=-1;
            else:
                next_link_no=g_vehicle_link_sequence[front_vehicle-1][current_link_idx+1];
            if(next_link_no<0):#this vehicle arrives at destination
                vehicle_vd_time[front_vehicle-1][current_link_idx]=t-1;
                vehicle_va_time[front_vehicle-1][current_link_idx+1]=t-1+g_link_travel_time[current_link_no-1];
                vehicle_vt_time[front_vehicle-1][current_link_idx+1]=t-1+g_link_travel_time[current_link_no-1];
                vehicle_vd_time[front_vehicle-1][current_link_idx+1]=t-1+g_link_travel_time[current_link_no-1];
                del link_queue[current_link_no-1][0]; # delete this vehicle from its current_link
                g_sim_vehicle_finish_time[front_vehicle-1] = t-1;
                g_sim_vehicle_complete_flag[front_vehicle-1] = 1;
            if(next_link_no>0):                        
                link_entering_flag=0;
                flowcapacity=min(LinkOutFlowCapacity[current_link_no-1][t],LinkInFlowCapacity[next_link_no-1][t]);
                if(LinkOutFlowCapacity[current_link_no-1][t]>=1 and LinkInFlowCapacity[next_link_no-1][t]>=1): # We first check if the link still has inbound capacity for this vehicle to enter at this time
                    link_entering_flag=1;
                elif (flowcapacity<1 and flowcapacity>0): # we need to toss a dice to determine if this vehicle should cross
                    g_seed=(g_seed * 16807.0) % 2147483647.0; #LGC method to generate a random variable
                    u = g_seed / 2147483647.0; #random number between 0 to 1
                    if (u < flowcapacity):
                        link_entering_flag=1;
                    else:
                        link_entering_flag=0;                    
                if link_entering_flag==1:# vehicle is allowed to leave
                    del link_queue[current_link_no-1][0]; # delete this vehicle from its current_link
                    link_temp_queue[next_link_no-1].append(front_vehicle);                        
                    #update the time matrix
                    vehicle_current_link_idx[front_vehicle-1]+=1;
                    vehicle_va_time[front_vehicle-1][current_link_idx+1]=t-1;
                    vehicle_vt_time[front_vehicle-1][current_link_idx+1]=t-1+g_link_travel_time[next_link_no-1];
                    vehicle_vd_time[front_vehicle-1][current_link_idx]=t-1;
                    vehicle_time_sequence[front_vehicle-1][current_link_idx+1]=t-1;
                    LinkInFlowCapacity[next_link_no-1][t]=LinkInFlowCapacity[next_link_no-1][t]-1;
                    LinkOutFlowCapacity[current_link_no-1][t]=LinkInFlowCapacity[current_link_no-1][t]-1;
                else:# vehicle is not allowed
                    break;
     #Step 3: Move vehicles from temp link queues to regular link queues. temp link is to ensure each vehicle is only updated once in Step 2 
    for l in range(1,g_number_of_links):            
        link_queue[l-1].extend(link_temp_queue[l-1]);
        link_temp_queue[l-1].clear();

################################
##Step 3: Tally Link Performance. In this example, we only update link travel times
###############################                            
experienced_TT=[1.0]*g_number_of_links;# allocate memory for saving
link_TT_sample_size=[0]*g_number_of_links; # to store the number of 
for v in g_agent_list:
    for idx  in range(0,len(g_vehicle_link_sequence[v-1])):
        link_no=g_vehicle_link_sequence[v-1][idx];
        TT=g_vehicle_va_time[v-1][idx+1]-g_vehicle_va_time[v-1][idx];
        link_TT_sample_size[link_no-1]=link_TT_sample_size[link_no-1]+1;
        experienced_TT[link_no-1]=experienced_TT[link_no-1]+TT; #total travel time is being calculated

for idx in range(0,g_number_of_links-1):
    if link_TT_sample_size[idx]>0:
        tt=experienced_TT[idx]/link_TT_sample_size[idx];
        experienced_TT[idx]=tt; # this is experienced TT
    else:
        experienced_TT[idx]=g_link_travel_time[idx];

############################################
##Step 4: Tally Link Performance. In this file, we get experienced link travel times
############################################ 
f=open('output_link_performance.csv','w',newline='');
csvwriter=csv.writer(f);
csvwriter.writerow(['Link_id,travel_time']);
for idx in range(0,g_number_of_links):
    line=[idx+1,experienced_TT[idx]];
    csvwriter.writerow(line);
f.close();

                            
                            
                    
                        
                        
                    
                
                
                
                
                
        
                
        
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

