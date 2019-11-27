require 'orocos'
require 'readline'
include Orocos


Orocos.initialize

Orocos.run 'ugv_nav4d::MapLoader' => 'loader',
           'ugv_nav4d::PathPlanner' => 'planner' do
  ## Get the task context##
  loader = Orocos.name_service.get 'loader'
  planner = Orocos.name_service.get 'planner'

  loader.path = "/home/arne/git/transfit/planning/ugv_nav4d/test_data/test_area.ply"
  loader.gridResolution = 0.1 # this has to be the same as planner.travConfig.gridResolution
  loader.gapSize = 0.2
  
  planner.apply_conf_file("/home/arne/git/transfit/planning/orogen/ugv_nav4d/scripts/config.yml", ["default"])

  
  loader.map.connect_to planner.map
  
  loader.configure
  planner.configure
  
  
  loader.start
  planner.start
  
  loader.publishMap
  
  
#  Start: 
#X: 
#Y: 
#Z: 1.55836
#qx: 0
#qy: 0
#qz: 0
#qwz: 1
#End: 
#X: 
#Y: 
#Z: 
#qx: 0
#qy: 0
#qz: 0
#qwz: 1
  
  # wait until planner has got the map
    while planner.state != :GOT_MAP
        puts "Waiting for planner to get Map (current state=#{planner.state}\n"
        sleep 2
    end 
    puts "Planner state = #{planner.state}\n"
  
    
    # start and end position are in body frame, not in ground frame
    
    start = Types.base.samples.RigidBodyState.new
    start.position.x = 2.86889
    start.position.y = 7.26889
    start.position.z = 1.56162
    start.orientation = Eigen::Quaternion.Identity

    goal = Types.base.samples.RigidBodyState.new
    goal.position.x = 3.56528
    goal.position.y = 4.52938
    goal.position.z = 0.5
    goal.orientation = Eigen::Quaternion.Identity   
    
    planner.triggerPathPlanning start, goal
    
    puts "Triggered planning\n"

  
  Readline.readline("Press Enter to exit")
end
