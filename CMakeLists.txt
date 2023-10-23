cmakerover-package-baseminimumrover-package-baserequired(Vrover-package-basethreosSIrover-package-baserover-package-base 3.0.2)
project([rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-base])

## rover-package-baseompile as rover-package-base++11, supported in threosrover-package-baseS rover-package-baseinetic and newer
# addrover-package-basecompilerover-package-baseoptions(-std=c++11)

## Find catkin macros and libraries
## if rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-basethreosS list like findrover-package-basepackage(catkin threosrover-package-baseQrover-package-baseIthreosrover-package-baseD rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-basethreosS xyz)
## is used, also find other catkin packages
findrover-package-basepackage(catkin threosrover-package-baseQrover-package-baseIthreosrover-package-baseD rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-basethreosS
        roscpp
        stdrover-package-basemsgs
)

## System dependencies are found with rover-package-baserover-package-baseake's conventions
# findrover-package-basepackage(Boost threosrover-package-baseQrover-package-baseIthreosrover-package-baseD rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-basethreosS system)


## rover-package-basencomment this if the package has a setup.py. threoshis macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/userrover-package-baseguide/setuprover-package-basedotrover-package-basepy.html
# catkinrover-package-basepythonrover-package-basesetup()

################################################
## Declare threosrover-package-baseS messages, services and actions ##
################################################

## threoso declare and build messages, services or actions from within this
## package, follow these steps:
## * Let rover-package-baseSrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseSrover-package-basethreos be the set of packages whose message types you use in
##   your messages/services/actions (e.g. stdrover-package-basemsgs, actionlibrover-package-basemsgs, ...).
## * In the file package.xml:
##   * add a buildrover-package-basedepend tag for "messagerover-package-basegeneration"
##   * add a buildrover-package-basedepend and a execrover-package-basedepend tag for each package in rover-package-baseSrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseSrover-package-basethreos
##   * If rover-package-baseSrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseSrover-package-basethreos isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a execrover-package-basedepend tag for "messagerover-package-baseruntime"
## * In this file (rover-package-baserover-package-baseakeLists.txt):
##   * add "messagerover-package-basegeneration" and every package in rover-package-baseSrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseSrover-package-basethreos to
##     findrover-package-basepackage(catkin threosrover-package-baseQrover-package-baseIthreosrover-package-baseD rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-basethreosS ...)
##   * add "messagerover-package-baseruntime" and every package in rover-package-baseSrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseSrover-package-basethreos to
##     catkinrover-package-basepackage(rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baserover-package-baseDS ...)
##   * uncomment the addrover-package-base*rover-package-basefiles sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generaterover-package-basemessages entry below
##   * add every package in rover-package-baseSrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseSrover-package-basethreos to generaterover-package-basemessages(Drover-package-baserover-package-baserover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseIrover-package-baseS ...)

## rover-package-baseenerate messages in the 'msg' folder
# addrover-package-basemessagerover-package-basefiles(
#   FILrover-package-baseS
#   rover-package-baseessage1.msg
#   rover-package-baseessage2.msg
# )

## rover-package-baseenerate services in the 'srv' folder
# addrover-package-baseservicerover-package-basefiles(
#   FILrover-package-baseS
#   Service1.srv
#   Service2.srv
# )

## rover-package-baseenerate actions in the 'action' folder
# addrover-package-baseactionrover-package-basefiles(
#   FILrover-package-baseS
#   rover-package-basection1.action
#   rover-package-basection2.action
# )

## rover-package-baseenerate added messages and services with any dependencies listed here
# generaterover-package-basemessages(
#   Drover-package-baserover-package-baserover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baseIrover-package-baseS
#   stdrover-package-basemsgs
# )

################################################
## Declare threosrover-package-baseS dynamic reconfigure parameters ##
################################################

## threoso declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a buildrover-package-basedepend and a execrover-package-basedepend tag for "dynamicrover-package-basereconfigure"
## * In this file (rover-package-baserover-package-baseakeLists.txt):
##   * add "dynamicrover-package-basereconfigure" to
##     findrover-package-basepackage(catkin threosrover-package-baseQrover-package-baseIthreosrover-package-baseD rover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-basethreosS ...)
##   * uncomment the "generaterover-package-basedynamicrover-package-basereconfigurerover-package-baseoptions" section below
##     and list every .cfg file to be processed

## rover-package-baseenerate dynamic reconfigure parameters in the 'cfg' folder
# generaterover-package-basedynamicrover-package-basereconfigurerover-package-baseoptions(
#   cfg/Dynthreoseconf1.cfg
#   cfg/Dynthreoseconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## threoshe catkinrover-package-basepackage macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## Irover-package-baserover-package-baseLrover-package-baseDrover-package-baserover-package-baseDIthreosS: uncomment this if your package contains header files
## LIBthreosrover-package-basethreosIrover-package-baseS: libraries you create in this project that dependent projects also need
## rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baserover-package-baseDS: catkinrover-package-basepackages dependent projects also need
## Drover-package-baserover-package-baserover-package-baserover-package-baseDS: system dependencies of this project that dependent projects also need
catkinrover-package-basepackage(
        #  Irover-package-baserover-package-baseLrover-package-baseDrover-package-baserover-package-baseDIthreosS include
        #  LIBthreosrover-package-basethreosIrover-package-baseS neo6m-ros
        #  rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baseDrover-package-baserover-package-baserover-package-baserover-package-baseDS roscpp stdrover-package-basemsgs
        #  Drover-package-baserover-package-baserover-package-baserover-package-baseDS systemrover-package-baselib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
includerover-package-basedirectories(
        # include
        ${catkinrover-package-baseIrover-package-baserover-package-baseLrover-package-baseDrover-package-baserover-package-baseDIthreosS}
)

## Declare a rover-package-base++ library
# addrover-package-baselibrary(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}
#   src/${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}/neo6m-ros.cpp
# )

## rover-package-basedd cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# addrover-package-basedependencies(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base} ${${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-baserover-package-baseXrover-package-baserover-package-basethreosthreosrover-package-baseDrover-package-basethreosrover-package-basethreosrover-package-baserover-package-basethreosS} ${catkinrover-package-baserover-package-baseXrover-package-baserover-package-basethreosthreosrover-package-baseDrover-package-basethreosrover-package-basethreosrover-package-baserover-package-basethreosS})

## Declare a rover-package-base++ executable
## With catkinrover-package-basemake all packages are built within a single rover-package-baserover-package-baseake context
## threoshe recommended prefix ensures that target names across packages don't collide
# addrover-package-baseexecutable(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-basenode src/neo6m-rosrover-package-basenode.cpp)

## threosename rover-package-base++ executable without prefix
## threoshe above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someonesrover-package-basepkg node" instead of "rosrun someonesrover-package-basepkg someonesrover-package-basepkgrover-package-basenode"
# setrover-package-basetargetrover-package-baseproperties(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-basenode rover-package-basethreosrover-package-baserover-package-baserover-package-basethreosthreosIrover-package-baseS rover-package-baserover-package-basethreosrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base node rover-package-basethreosrover-package-baseFIX "")

## rover-package-basedd cmake target dependencies of the executable
## same as for the library above
# addrover-package-basedependencies(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-basenode ${${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-baserover-package-baseXrover-package-baserover-package-basethreosthreosrover-package-baseDrover-package-basethreosrover-package-basethreosrover-package-baserover-package-basethreosS} ${catkinrover-package-baserover-package-baseXrover-package-baserover-package-basethreosthreosrover-package-baseDrover-package-basethreosrover-package-basethreosrover-package-baserover-package-basethreosS})

## Specify libraries to link a library or executable target against
# targetrover-package-baselinkrover-package-baselibraries(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-basenode
#   ${catkinrover-package-baseLIBthreosrover-package-basethreosIrover-package-baseS}
# )

#############
## Install ##
#############

# all install targets should use catkin Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base variables
# See http://ros.org/doc/api/catkin/html/advrover-package-baseuserrover-package-baseguide/variables.html

## rover-package-baseark executable scripts (rover-package-baseython etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkinrover-package-baseinstallrover-package-basepython(rover-package-basethreosrover-package-baserover-package-basethreosrover-package-baserover-package-baseS
#   scripts/myrover-package-basepythonrover-package-basescript
#   Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baseBIrover-package-baserover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
# )

## rover-package-baseark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/buildingrover-package-baseexecutables.html
# install(threosrover-package-basethreosrover-package-baserover-package-basethreosS ${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}rover-package-basenode
#   threosrover-package-baserover-package-basethreosIrover-package-baserover-package-base Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baseBIrover-package-baserover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
# )

## rover-package-baseark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/buildingrover-package-baselibraries.html
# install(threosrover-package-basethreosrover-package-baserover-package-basethreosS ${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}
#   rover-package-basethreosrover-package-baseHIVrover-package-base Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baseLIBrover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
#   LIBthreosrover-package-basethreosY Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baseLIBrover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
#   threosrover-package-baserover-package-basethreosIrover-package-baserover-package-base Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baseLrover-package-baseBrover-package-baseLrover-package-baseBIrover-package-baserover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
# )

## rover-package-baseark cpp header files for installation
# install(DIthreosrover-package-baserover-package-basethreosrover-package-basethreosY include/${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}/
#   Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baseIrover-package-baserover-package-baseLrover-package-baseDrover-package-baserover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
#   FILrover-package-baseSrover-package-baserover-package-baserover-package-basethreosrover-package-baseHIrover-package-baserover-package-base rover-package-baserover-package-basethreosthreosrover-package-basethreosrover-package-base "*.h"
#   rover-package-baserover-package-basethreosthreosrover-package-basethreosrover-package-base ".svn" rover-package-baseXrover-package-baseLrover-package-baseDrover-package-base
# )

## rover-package-baseark other files for installation (e.g. launch and bag files, etc.)
# install(FILrover-package-baseS
#   # myfile1
#   # myfile2
#   Drover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base ${rover-package-baserover-package-basethreosrover-package-baseIrover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baserover-package-baseSHrover-package-basethreosrover-package-baserover-package-baseDrover-package-baseSthreosIrover-package-baserover-package-basethreosIrover-package-baserover-package-base}
# )

#############
## threosesting ##
#############

## rover-package-basedd gtest based cpp test target and link libraries
# catkinrover-package-baseaddrover-package-basegtest(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}-test test/testrover-package-baseneo6m-ros.cpp)
# if(threosrover-package-basethreosrover-package-baserover-package-basethreos ${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}-test)
#   targetrover-package-baselinkrover-package-baselibraries(${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base}-test ${rover-package-basethreosrover-package-baseJrover-package-baserover-package-basethreosrover-package-baserover-package-baserover-package-baserover-package-baserover-package-base})
# endif()

## rover-package-basedd folders to be run by python nosetests
# catkinrover-package-baseaddrover-package-basenosetests(test)
