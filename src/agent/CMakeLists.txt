ADD_SUBDIRECTORY( loc )
ADD_SUBDIRECTORY( world )
ADD_SUBDIRECTORY( integrator )

add_executable ( agent
    strategy/Formation.cpp
    strategy/Strategy.cpp
    strategy/StrategyParser.cpp
    DriveVector.cpp
    Decision.cpp
    Cambada.cpp
	main.cpp
    
    # Controller list
    controllers/Controller.cpp
    controllers/CMove
    controllers/CArc
    controllers/CRotateAroundBall
    controllers/CRotate
    
    # Behaviours
    behaviours/Behaviour
    behaviours/CambadaArbitrator
    
    behaviours/BallHandling/BKickToTheirGoal
    
    behaviours/General/BStop
    behaviours/General/BStopRobotGS
    
    behaviours/WithoutBall/BSearchBall
    behaviours/WithoutBall/BGoToVisibleBall
    
    # Roles
    roles/Role
    roles/RoleStop
    roles/RoleStriker
)
TARGET_LINK_LIBRARIES( agent
	m
	integrator
	localization
	filters
	util
	loc
	rtdb
	pman
	worldstate
	geom
	tcod
	tcodxx
	xerces-c
#	rcsc_agent rcsc_ann rcsc_net rcsc_time rcsc_param rcsc_gz rcsc_rcg rcsc_geom
)
