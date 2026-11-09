; Test 4: three robots select and execute one action each in the same round.
(define (problem p4-network)
    (:domain MultiRobotRunner)
    (:objects
        robot1 robot2 robot3 - robot
        square1 square2 square3 square4 square5 square6 - gridsquare
        north east south west - direction
        c0 c1 c2 c3 - counter
    )

    (:init
        ; Robot locations and rotations
        (robot-at robot1 square1)
        (robot-rotated robot1 east)
        (robot-at robot2 square3)
        (robot-rotated robot2 south)
        (robot-at robot3 square4)
        (robot-rotated robot3 east)

        ; A two-by-three grid
        (gridsquare-connected square1 square2 east)
        (gridsquare-connected square2 square1 west)
        (gridsquare-connected square2 square3 east)
        (gridsquare-connected square3 square2 west)
        (gridsquare-connected square4 square5 east)
        (gridsquare-connected square5 square4 west)
        (gridsquare-connected square5 square6 east)
        (gridsquare-connected square6 square5 west)
        (gridsquare-connected square1 square4 south)
        (gridsquare-connected square4 square1 north)
        (gridsquare-connected square2 square5 south)
        (gridsquare-connected square5 square2 north)
        (gridsquare-connected square3 square6 south)
        (gridsquare-connected square6 square3 north)

        ; Clockwise and anti-clockwise 90-degree rotations
        (rotation-adjacent north east)
        (rotation-adjacent east south)
        (rotation-adjacent south west)
        (rotation-adjacent west north)
        (rotation-adjacent east north)
        (rotation-adjacent south east)
        (rotation-adjacent west south)
        (rotation-adjacent north west)

        ; Initial round state for three robots
        (choosing)
        (unplanned robot1)
        (unplanned robot2)
        (unplanned robot3)
        (remaining c3)
        (robot-count c3)
        (next-count c0 c1)
        (next-count c1 c2)
        (next-count c2 c3)
        (zero c0)
    )

    (:goal (and
        (robot-at robot1 square2)
        (robot-at robot2 square6)
        (robot-at robot3 square5)
        (choosing)
    ))
)
