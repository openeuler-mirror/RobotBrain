-- rttlua-gnulinux -i modules/deployment/robot_deployer.lua 

function sleep(n)
   local t0 = os.clock()
   while os.clock() - t0 <= n do end
end

print(package.cpath)
package.cpath = "/usr/local/lib/?.so;"..package.cpath
print(package.cpath)
core = require("libcore")
core.Application.Init("./data/test_data/robot/")
require("rttlib")
rttlib.color = true -- Since we're running this script with -i

package.loadlib("librobot_exception.so", 0)
package.loadlib("libcore.so", 0)
package.loadlib("libexception.so", 0)
package.loadlib("librobot_planning.so", 0)
package.loadlib("librobot_teach.so", 0)

-- -- Get our Lua TC
tc = rtt:getTC()
rttlib.stat()
rttlib.info()
-- -- Get the deployer
deployer = tc:getPeer("Deployer")

deployer:import("robot_brain_cpp")
print("-------------------------Components----------------------")
deployer:displayComponentTypes()
print("---------------------------------------------------------")

deployer:loadComponent("ethercat", "rosc::EthercatComponent")
deployer:loadComponent("robot", "rosc::RobotComponent")
deployer:loadComponent("robot_service", "rosc::RobotServiceComponent")
deployer:loadComponent("trajectory", "rosc::TrajectoryComponent")
deployer:loadComponent("grpc_service", "rosc::GrpcServiceComponent")
ethercat = deployer:getPeer("ethercat")
robot = deployer:getPeer("robot")
robot_service = deployer:getPeer("robot_service")
trajectory = deployer:getPeer("trajectory")
grpc_service = deployer:getPeer("grpc_service")


deployer:connectPeers("ethercat", "robot")
deployer:connectPeers("trajectory", "robot")
deployer:connectPeers("robot_service", "trajectory")

deployer:connectPorts("ethercat", "robot")
deployer:connectPorts("trajectory", "robot")
deployer:connectPorts("robot_service", "trajectory")
deployer:connectPorts("grpc_service", "robot_service");

ethercat:configure()
trajectory:configure()
robot:configure()
robot_service:configure()
grpc_service:configure()

ethercat:start()
trajectory:start()
robot:start()
robot_service:start()
sleep(3)
grpc_service:start()
grpc_service:StartService()

rttlib.stat()
