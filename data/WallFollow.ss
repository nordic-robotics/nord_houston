#define front_min 0.1f
#define side_min 0.1f
#define encoder_min 0.1f
#define normal_speed 0.5f

#var float delta_encoder_left
#var float delta_encoder_right
#var float ir_front
#var float ir_back
#var float ir_left_front
#var float ir_left_back
#var float ir_right_front
#var float ir_right_back

selector WallFollow:
    sequence Stuck:
        cond delta_encoder_left < #encoder_min
        cond delta_encoder_right < #encoder_min
        call publish(action::backwards_distance, 0.1f)
    sequence FrontWall:
        cond ir_front < #front_min
        selector Turn:
            sequence TurnLeft:
                cond ir_right_front + ir_right_back < ir_left_front + ir_left_back
                call publish(action::turn_left, 90.0f)
            sequence TurnRight:
                cond ir_left_front + ir_left_back < ir_right_front + ir_right_back
                call publish(action::turn_right, 90.0f)
    call publish(action::forwards_speed, #normal_speed)
