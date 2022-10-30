import carla

class RoadTopology:
    """
    class for generating chosen circuit's road topology,
    topology is saved with waypoins list
    """
    def __init__(self,world,sampling_resolution=1000.0) -> None:
        self.sampling_resolution=sampling_resolution
        circuit=world.get_map().get_spawn_points()

        

        #code for simulation road generation
        self.circuit_topology=[]
        roads={8,11,0,40,41,1,61,62,2,117,118,3,13,15,20,5,93,94,6,157,158,7,14} #road id set for chosen roads

        for waypoint_tuple in circuit:
            start_waypoint=waypoint_tuple[0]
            end_waypoint=waypoint_tuple[1]
            if start_waypoint.road_id in roads and end_waypoint.road_id in roads:
                #begin_label=str(start_waypoint.road_id)+'*'+str(start_waypoint.section_id)+'*'+str(start_waypoint.lane_id)
                #end_label=str(end_waypoint.road_id)+'*'+str(end_waypoint.section_id)+'*'+str(end_waypoint.lane_id)
                self.circuit_topology.append((start_waypoint,end_waypoint))
                self._waypoint_generate(self.circuit_topology,start_waypoint,end_waypoint,distance=self.sampling_resolution)
                self.circuit_topology.remove((start_waypoint,end_waypoint))

        self.circuit_waypoints=set()
        for wapoint_tuple in self.circuit_topology:
            self.circuit_waypoints.add(wapoint_tuple[0])
            self.circuit_waypoints.add(waypoint_tuple[1])

    def get_topology(self):
        return list(self.circuit_waypoints)

    def _waypoint_generate(self,topology,start_wp,end_wp,distance):
        end_waypoint=end_wp   
        start_waypoint=start_wp
        endloc=end_waypoint.transform.location
        sampling_resolution=distance
        #count=0

        former_wp=start_waypoint
        if former_wp.transform.location.distance(endloc)>sampling_resolution:
            latter_wp=former_wp.next(sampling_resolution)[0]
            while latter_wp.transform.location.distance(endloc)>sampling_resolution:
                # if count!=0:
                #     former_label=str(former_wp.road_id)+'*'+str(former_wp.section_id)+'*'+str(former_wp.lane_id)+'*'+str(count-1)
                # else:
                #     former_label=str(former_wp.road_id)+'*'+str(former_wp.section_id)+'*'+str(former_wp.lane_id)
                #latter_label=str(latter_wp.road_id)+'*'+str(latter_wp.section_id)+'*'+str(latter_wp.lane_id)+'*'+str(count)
                topology.append((former_wp,latter_wp))
                former_wp=latter_wp
                latter_wp=latter_wp.next(sampling_resolution)[0]
                #count+=1
                #print(count,end='\t')
        # if count!=0:
        #     former_label=str(former_wp.road_id)+'*'+str(former_wp.section_id)+'*'+str(former_wp.lane_id)+'*'+str(count-1)
        # else:
        #     former_label=str(former_wp.road_id)+'*'+str(former_wp.section_id)+'*'+str(former_wp.lane_id)
        #end_label=str(end_waypoint.road_id)+'*'+str(end_waypoint.section_id)+'*'+str(end_waypoint.lane_id)
        topology.append((former_wp,end_waypoint))
        #print(start_waypoint.road_id,end_waypoint.road_id,count,end='\t')
        #print(start_waypoint.transform.location,end_waypoint.transform.location,end='\t')       