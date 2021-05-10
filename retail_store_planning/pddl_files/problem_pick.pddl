(define (problem waypoint_following)
    (:domain waypoint_following)
    (:requirements :strips :typing)

    (:objects   tiago - robot
                leftgrip rightgrip - gripper
                wp0 wp_table_1 wp_table_2 wp_table_3 - waypoint
                aruco_cube_111 aruco_cube_222 aruco_cube_333 aruco_cube_444 aruco_cube_582 - object

    )
    
    (:init
    (visited wp0)
    (robot-at tiago wp0)
    (free rightgrip)
    (free leftgrip)
    (object-at aruco_cube_444 wp_table_3)
    (object-at aruco_cube_333 wp_table_3)
    (object-at aruco_cube_222 wp_table_2)
    (object-at aruco_cube_111 wp_table_2)
    (object-at aruco_cube_582 wp_table_1)
    
    )
    
    (:goal 
    (is_holding rightgrip aruco_cube_444) 

	)

)
