# https://pythonprogramming.altervista.org/raycasting-with-pygame/

def cast_rays():
    start_angle = player_angle - HALF_FOV
    
    for ray in range(CASTED_RAYS):
        for depth in range(MAX_DEPTH):
            target_x = player_x - math.sin(start_angle) * depth
            target_y = player_y + math.cos(start_angle) * depth
            col = int(target_x / TILE_SIZE)
            row = int(target_y / TILE_SIZE)
 
            square = row * MAP_SIZE + col
            (target_y / TILE_SIZE) * MAP_SIZE + target_x / TILE_SIZE 
            if MAP[square] == '#':
                pygame.draw.rect(win,(0,255,0),(col * TILE_SIZE,
                                                row * TILE_SIZE,
                                                TILE_SIZE - 2,
                                                TILE_SIZE - 2))
                pygame.draw.line(win, (255,255,0),(player_x,player_y),(target_x,target_y))
                color = 50 / (1 + depth * depth * 0.0001)
                
                depth *= math.cos(player_angle - start_angle)
                    
                wall_height = 21000 / (depth + 0.0001)
                
                if wall_height > SCREEN_HEIGHT: wall_height == SCREEN_HEIGHT
                
                pygame.draw.rect(win,(color,color,color), (SCREEN_HEIGHT + ray * SCALE,(SCREEN_HEIGHT / 2) - wall_height / 2,SCALE,wall_height))
                
                break
    
        start_angle += STEP_ANGLE