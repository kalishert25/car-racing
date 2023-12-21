use std::collections::VecDeque;

use notan::draw::*;

use notan::prelude::*;
use rand::Rng;

#[derive(AppState)]
struct State {
    map: Map,
}

#[notan_main]
fn main() -> Result<(), String> {
    notan::init_with(setup)
        .add_config(DrawConfig)
        .update(update)
        .draw(draw)
        .build()
}
fn update(app: &mut App, state: &mut State) {
    
    let (x, y) = app.mouse.position();
    let (x, y) = (
        (x / state.map.tile_size) as i32,
        (y / state.map.tile_size) as i32,
    );
    if app.mouse.was_pressed(MouseButton::Left) {
       //move car to mouse position
        state.map.cars[0].x = x;
        state.map.cars[0].y = y;
    }
}
fn setup(gfx: &mut Graphics) -> State {
    let car = Car::new(0, 0, 0, 0);
    let mut map = Map::new(100, 100, 8.0);
    map.cars.push(car);
    let log = map.create_random_racetrack(10., DirectionOfTravel::Clockwise);
    println!("amp: {}, noise: {}, f1: {}, shift1: {}, shift2: {}, ar: {}, height: {}, width: {}, radius_offset: {}, thickness: {}", log.0, log.1, log.2, log.3, log.4, log.5, log.6, log.7, log.8, log.9);
    //println!("start direction: {:?}", map.start_dir);
    //map.print_map();
    State { map }
}

fn draw(gfx: &mut Graphics, state: &mut State) {
    let map = &state.map;
    let mut draw = gfx.create_draw();
    draw.clear(Color::BLACK);
    for x in 0..map.get_width() {
        for y in 0..map.get_height() {
            draw.rect(
                (x as f32 * map.tile_size, y as f32 * map.tile_size),
                (map.tile_size, map.tile_size),
            )
            .color(map.get_tile(x, y).color());
            //if its a road, draw a grid dot in the middle
            if map.get_tile(x, y) == &Tile::Road {
                draw.rect(
                    (
                        x as f32 * map.tile_size + map.tile_size / 2.0,
                        y as f32 * map.tile_size + map.tile_size / 2.0,
                    ),
                    (map.tile_size / 8.0, map.tile_size / 8.0),
                )
                .color(Color::BLACK);
            }
        }
    }
    //for each car, draw it
    for car in map.cars.iter() {
        draw.rect(
            (car.x as f32 * map.tile_size, car.y as f32 * map.tile_size),
            (map.tile_size, map.tile_size),
        )
        .color(Color::RED);
    }
    // let mut draw = gfx.create_draw();
    // draw.clear(Color::BLACK);
    // draw.triangle((400.0, 100.0), (100.0, 500.0), (700.0, 500.0));
    // draw.rect((200., 300.), (40., 40.));
    gfx.render(&draw);
}

// Map struct that will hold the data for the race track. Consists of a 2D array of tiles, represented as a 1d array of Tiles, and the width and height of the map.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Direction {
    Up,
    Down,
    Left,
    Right,
    None,
}
impl Direction {
    fn vector(&self) -> (i32, i32) {
        match self {
            Direction::Up => (0, -1),
            Direction::Down => (0, 1),
            Direction::Left => (-1, 0),
            Direction::Right => (1, 0),
            Direction::None => (0, 0),
        }
    }
    fn opposite(&self) -> Direction {
        match self {
            Direction::Up => Direction::Down,
            Direction::Down => Direction::Up,
            Direction::Left => Direction::Right,
            Direction::Right => Direction::Left,
            Direction::None => Direction::None,
        }
    }
}
#[derive(Debug)]
struct Map {
    tiles: Vec<Tile>,
    width: i32,
    height: i32,
    tile_size: f32,
    start_dir: Direction,
    cars: Vec<Car>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tile {
    Road,
    Wall,
    Finish,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DirectionOfTravel {
    Clockwise,
    CounterClockwise,
}
//each tile has a different color associated with it
impl Tile {
    fn color(&self) -> Color {
        match self {
            Tile::Road => Color::WHITE,
            Tile::Wall => Color::BLACK,
            Tile::Finish => Color::GREEN,
        }
    }
}

impl Map {
    // Creates a new map with the given width and height.
    fn new(width: i32, height: i32, tile_size: f32) -> Map {
        let mut tiles = Vec::new();
        for _ in 0..width * height {
            tiles.push(Tile::Road);
        }
        Map {
            tiles,
            width,
            height,
            tile_size,
            start_dir: Direction::None,
            cars: Vec::new(),
        }
    }

    // Returns the tile at the given x and y coordinates.
    fn get_tile(&self, x: i32, y: i32) -> &Tile {
        &self.tiles[(y * self.width + x) as usize]
    }

    // Sets the tile at the given x and y coordinates to the given tile.
    fn set_tile(&mut self, x: i32, y: i32, tile: Tile) {
        self.tiles[(y * self.width + x) as usize] = tile;
    }

    // Returns the width of the map.
    fn get_width(&self) -> i32 {
        self.width
    }

    // Returns the height of the map.
    fn get_height(&self) -> i32 {
        self.height
    }
    fn create_random_racetrack(
        &mut self,
        radius_offset: f32,
        dot: DirectionOfTravel,
    ) -> (f32, f32, f32, f32, f32, f32, i32, i32, f32, f32) {
        //link to desmos graph: https://www.desmos.com/calculator/rxjdaxwqxm
        //first set all tiles to walls
        for x in 0..self.get_width() {
            for y in 0..self.get_height() {
                self.set_tile(x, y, Tile::Wall);
            }
        }
        //now we have to hollow out the track
        //lets make an circle to start with
        let (amp1, noise, f1, thickness, shift1, shift2, ar) = self.hollow_out_path(radius_offset);
        //now we place the finish line.

        let (x, y) = self.pick_random_tile_type(Tile::Road);
        //now we look horizontally and vertically for the nearest wall tile. Whichever is a shorter line is the one we will use
        let mut horizontal_line = vec![(x, y)];
        let mut vertical_line = vec![(x, y)];
        //look left and right
        let mut curr_x = x;
        loop {
            curr_x -= 1;
            if curr_x < 0 || self.get_tile(curr_x, y) == &Tile::Wall {
                break;
            }
            horizontal_line.push((curr_x, y));
        }
        curr_x = x;
        loop {
            curr_x += 1;
            if curr_x >= self.get_width() || self.get_tile(curr_x, y) == &Tile::Wall {
                break;
            }
            horizontal_line.push((curr_x, y));
        }
        //look up and down
        let mut curr_y = y;
        loop {
            curr_y -= 1;
            if curr_y < 0 || self.get_tile(x, curr_y) == &Tile::Wall {
                break;
            }
            vertical_line.push((x, curr_y));
        }
        curr_y = y;
        loop {
            curr_y += 1;
            if curr_y >= self.get_height() || self.get_tile(x, curr_y) == &Tile::Wall {
                break;
            }
            vertical_line.push((x, curr_y));
        }
        //now we have to pick the shorter line
        let shorter_line = if horizontal_line.len() < vertical_line.len() {
            //start direction is either up or down depending on if the track is clockwise or counterclockwise
            self.start_dir = if dot == DirectionOfTravel::Clockwise {
                Direction::Up
            } else {
                Direction::Down
            };
            horizontal_line
        } else {
            //start direction is either left or right depending on if the track is clockwise or counterclockwise
            self.start_dir = if dot == DirectionOfTravel::Clockwise {
                Direction::Left
            } else {
                Direction::Right
            };
            vertical_line
        };
        //small detail, any part of the finish line that is adjacent to more than one wall tile should be a wall tile
        for (x, y) in shorter_line.iter() {
            //count the number of wall tiles adjacent to this tile
            let mut num_walls = 0;
            for (xx, yy) in vec![(1, 0), (-1, 0), (0, 1), (0, -1)].iter() {
                if x + xx >= 0
                    && x + xx < self.width
                    && y + yy >= 0
                    && y + yy < self.height
                    && self.get_tile(x + xx, y + yy) == &Tile::Wall
                {
                    num_walls += 1;
                }
            }

            //println!("(x, y): ({}, {}), num_walls: {}", x, y, num_walls);
            if num_walls > 1 {
                self.set_tile(*x, *y, Tile::Wall);
            } else {
                self.set_tile(*x, *y, Tile::Finish);
            }
        }
        //if the finish line is such that there is no other path to the finish line, we have to pick a new map
        //we will do this by checking if there is a path from the finish line back to the finish line in the opposite direction
        if !self.is_valid_finish_line_placement(&shorter_line) {
            return self.create_random_racetrack(radius_offset - 3.0, dot);
        }
        //return all the parameters that we used to generate the track
        (
            amp1,
            noise,
            f1,
            shift1,
            shift2,
            ar,
            self.height,
            self.width,
            radius_offset,
            thickness,
        )
    }

    fn hollow_out_path(&mut self, radius_offset: f32) -> (f32, f32, f32, f32, f32, f32, f32) {
        let mut rng = rand::thread_rng();

        //amplitude can be positive or negative
        let amp1 = if rng.gen_bool(0.5) {
            rng.gen_range(0.5..2.0)
        } else {
            -rng.gen_range(0.5..2.0)
        };
        let amp2 = if rng.gen_bool(0.5) {
            rng.gen_range(0.5..2.0)
        } else {
            -rng.gen_range(0.5..2.0)
        };
        //pick a value between 2 and 3 for the craziness of the track
        let noise = rng.gen_range(2.0..3.0);
        //f1 is the frequency of the sine wave
        //f can be positive or negative
        let f1 = if rng.gen_bool(0.5) {
            rng.gen_range(1.3..2.7)
        } else {
            -rng.gen_range(1.3..2.7)
        };
        let f2 = if rng.gen_bool(0.5) {
            rng.gen_range(1.3..2.7)
        } else {
            -rng.gen_range(1.3..2.7)
        };

        let thickness = rng.gen_range(0.8..2.0);
        //pick a random phase shift for the sine wave, between 0 and 2pi
        let shift1 = rng.gen_range(0.0..std::f32::consts::PI * 2.0);
        let shift2 = rng.gen_range(0.0..std::f32::consts::PI * 2.0);
        //aspect ratio is not allowed to be more than 3:1 or 1:3
        let ar = (self.get_width() as f32 / self.get_height() as f32)
            .max(1.0 / 3.0)
            .min(3.0);

        let radius = (self.get_width() / 2).max(self.get_height() / 2) as f32 - radius_offset;
        let center_x = self.get_width() as f32 / 2.0;
        let center_y = self.get_height() as f32 / 2.0;
        //now we have to loop through all the tiles and set them to road if they are in the big circle but not in the small circle
        for x in 0..self.get_width() {
            for y in 0..self.get_height() {
                let x_l = noise * (x as f32 - center_x) / radius;
                let y_l = noise * (y as f32 - center_y) / radius;

                let wave_lx = amp1 * ((f1 * x_l + shift1).sin() + 1.05).sqrt();
                let wave_ly = amp2 * ((f2 * y_l + shift2).sin() + 1.05).sqrt();

                let dist_l =
                    (x_l).powi(2) + ar * ar * (y_l).powi(2) + wave_lx + wave_ly - (noise * noise);

                //println!("distL: {}, distS: {} ", dist_l, dist_s);
                if dist_l > -thickness * noise && dist_l < 0.0 {
                    self.set_tile(x, y, Tile::Road);
                }
            }
        }
        (amp1, noise, f1, thickness, shift1, shift2, ar)
    }
    //function to pick a random tile of a certain type
    fn pick_random_tile_type(&self, tile_type: Tile) -> (i32, i32) {
        let mut rng = rand::thread_rng();
        let tiles_of_type: Vec<(i32, i32)> = self
            .tiles
            .iter()
            .enumerate()
            .filter(|(_, tile)| **tile == tile_type)
            .map(|(index, _)| (index as i32 % self.width, index as i32 / self.width))
            .collect();
        return tiles_of_type[rng.gen_range(0..tiles_of_type.len())];
    }
    //function to print the map to the console for debugging "." is road and "#" is wall
    fn print_map(&self) {
        for y in 0..self.get_height() {
            for x in 0..self.get_width() {
                match self.get_tile(x, y) {
                    Tile::Road => print!("."),
                    Tile::Wall => print!("#"),
                    Tile::Finish => print!("F"),
                }
            }
            println!();
        }
    }
    fn is_valid_finish_line_placement(&self, potential_finish_line: &Vec<(i32, i32)>) -> bool {
        //first check if the finish line is too long or too short
        //too long = more than 1/3 of the map
        if potential_finish_line.len() == 0
            || potential_finish_line.len() > i32::min(self.width, self.height) as usize / 3
        {
            return false;
        }
        //bfs
        let mut queue: VecDeque<(i32, i32)> = std::collections::VecDeque::new();
        let mut visited: Vec<bool> =
            vec![false; self.get_width() as usize * self.get_height() as usize];
        //start next to the finish line, go one tile in the start direction
        let start_dir = self.start_dir.vector();
        for (x, y) in potential_finish_line.iter() {
            visited[(y * self.width + x) as usize] = true;
            let (x, y) = (x + start_dir.0, y + start_dir.1);
            queue.push_back((x, y));
            visited[(y * self.width + x) as usize] = true;
        }
        while let Some((x, y)) = queue.pop_front() {
            //check if we are at the finish line
            //we are only allowed to cross the finish line in the same direction as the start direction. This prevents us from going backwards and crossing the finish line trivially.
            let neighbors = vec![(1, 0), (-1, 0), (0, 1), (0, -1)];
            for (xx, yy) in neighbors.iter() {
                //check if we're in  bounds
                if x + xx < 0 || x + xx >= self.width || y + yy < 0 || y + yy >= self.height {
                    continue;
                }
                if self.get_tile(x + xx, y + yy) == &Tile::Finish
                    && Direction::vector(&self.start_dir) == (*xx, *yy)
                {
                    return true;
                }
                if self.get_tile(x + xx, y + yy) == &Tile::Road
                    && !visited[((y + yy) * self.width + x + xx) as usize]
                {
                    queue.push_back((x + xx, y + yy));
                    visited[((y + yy) * self.width + x + xx) as usize] = true;
                }
            }
        }
        return false;
    }
}

#[derive(Debug, Eq, PartialEq, Clone)]
struct Car {
    x: i32,
    y: i32,
    vx: i32,
    vy: i32,
}
impl Car {
    fn new(x: i32, y: i32, vx: i32, vy: i32) -> Car {
        Car { x, y, vx, vy }
    }

}
