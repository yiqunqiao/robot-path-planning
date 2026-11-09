; Test 2: two robots exchange ends of a corridor.
; The side branch gives one robot a safe detour, so the multi-robot plan
; is longer than its direct single-robot path.
(define (problem p2-network)
    (:domain MultiRobotRunner)
    (:objects
        robot1 robot2 - robot
        square1 square2 square3 square4 square5 - gridsquare
        north east south west - direction
        c0 c1 c2 - counter
    )

    (:init
        ; Robot locations and rotations
        (robot-at robot1 square1)
        (robot-rotated robot1 east)
        (robot-at robot2 square3)
        (robot-rotated robot2 west)

        ; Corridor square1-square2-square3 with a side branch
        (gridsquare-connected square1 square2 east)
        (gridsquare-connected square2 square1 west)
        (gridsquare-connected square2 square3 east)
        (gridsquare-connected square3 square2 west)
        (gridsquare-connected square2 square4 south)
        (gridsquare-connected square4 square2 north)
        (gridsquare-connected square4 square5 south)
        (gridsquare-connected square5 square4 north)

        ; Clockwise and anti-clockwise 90-degree rotations
        (rotation-adjacent north east)
        (rotation-adjacent east south)
        (rotation-adjacent south west)
        (rotation-adjacent west north)
        (rotation-adjacent east north)
        (rotation-adjacent south east)
        (rotation-adjacent west south)
        (rotation-adjacent north west)

        ; Initial round state for two robots
        (choosing)
        (unplanned robot1)
        (unplanned robot2)
        (remaining c2)
        (robot-count c2)
        (next-count c0 c1)
        (next-count c1 c2)
        (zero c0)
    )

    (:goal (and
        (robot-at robot1 square3)
        (robot-at robot2 square1)
        (choosing)
    ))
)
