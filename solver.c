/*
 * solver.c - Team 3 - robot name: Jinx          (ONE FILE, nothing else to add)
 *
 * Flow:
 *   1. flood fill search to the centre
 *   2. visit every other cell (builds the complete map)
 *   3. route back to the start over the known map
 *   4. route to the centre over the known map  = the speed run
 *
 * Sections in this file, top to bottom:
 *   A. Settings
 *   B. MAP + LOCATION   walls (4 bits per cell, both sides), explored flags,
 *                       x / y / heading, sensing, turning, moving
 *   C. FLOOD FILL       exact start values (BFS), queue re-flood, move choice
 *   D. SPEED RUN        shortest known route + driving it
 *   E. SEARCH + MAIN    solver_run()
 *
 * Rules kept: only mouse_hal.h and standard C headers, everything through
 * mm_... functions, no printf (mm_log instead), maze size and goal read
 * from mm_maze(). Builds unchanged for the simulator and the ESP32.
 */
#include "mouse_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ======================================================================== */
/* A. Settings                                                               */
/* ======================================================================== */
#define INF 9999                   /* "cannot reach the target"               */

#ifndef TURN_COST
#define TURN_COST 1                /* extra cost of a 90 degree turn (cells);  */
#endif                             /* 0 = plain fewest-cells path              */

#ifndef MERGE_STRAIGHTS
#define MERGE_STRAIGHTS 1          /* 1 = equal directions become one straight;*/
#endif                             /* 0 = one cell at a time                   */

#ifndef EXPLORE_MODE
#define EXPLORE_MODE 1             /* what to do after reaching the centre:    */
#endif                             /*  0 = nothing, go straight back           */
                                   /*  1 = nearest unexplored cell first       */
                                   /*      (fewest repeated paths and turns)    */
                                   /*  2 = row by row (simple, more travel)    */

/* ======================================================================== */
/* B. MAP + LOCATION                                                         */
/* ======================================================================== */
static uint8_t walls[MM_MAX_SIZE][MM_MAX_SIZE];
static uint8_t known[MM_MAX_SIZE][MM_MAX_SIZE];
static bool    explored[MM_MAX_SIZE][MM_MAX_SIZE];

static int x, y, heading;
static mm_maze_t maze;

static const int dx[4] = {0, 1, 0, -1};
static const int dy[4] = {1, 0, -1, 0};

static void observe_side(int wx, int wy, int dir, bool has_wall);

static void map_init(void)
{
    maze = mm_maze();                 /* never hard-code size or goal */
    memset(walls, 0, sizeof(walls));
    memset(known, 0, sizeof(known));
    memset(explored, 0, sizeof(explored));

    /* The outer border is always a wall: record it before the first move,
     * so cells whose border side is never sensed (like the start cell's
     * south side) still match the real maze in the map dump. */
    for (int cx = 0; cx < maze.width; cx++) {
        observe_side(cx, 0, MM_SOUTH, true);
        observe_side(cx, maze.height - 1, MM_NORTH, true);
    }
    for (int cy = 0; cy < maze.height; cy++) {
        observe_side(0, cy, MM_WEST, true);
        observe_side(maze.width - 1, cy, MM_EAST, true);
    }

    x = 0;
    y = 0;
    heading = MM_NORTH;
}

static bool in_bounds(int cx, int cy)
{
    return cx >= 0 && cx < maze.width && cy >= 0 && cy < maze.height;
}

static bool in_box(int cx, int cy, int x0, int y0, int x1, int y1)
{
    return cx >= x0 && cx <= x1 && cy >= y0 && cy <= y1;
}

/* Record a wall on BOTH cells that share it. The neighbour update is
 * skipped for border cells (no neighbour outside the maze). */
static void set_wall(int wx, int wy, int dir)
{
    if (!in_bounds(wx, wy)) {
        return;
    }

    if (!(walls[wx][wy] & MM_WALL(dir))) {
        walls[wx][wy] |= MM_WALL(dir);
        mm_show_wall(wx, wy, dir);
    }

    int nx = wx + dx[dir];
    int ny = wy + dy[dir];
    if (in_bounds(nx, ny)) {
        walls[nx][ny] |= MM_WALL((dir + 2) & 3);
    }
}

/* Mark a side as observed (on both cells), and store the wall if present. */
static void observe_side(int wx, int wy, int dir, bool has_wall)
{
    if (!in_bounds(wx, wy)) {
        return;
    }

    known[wx][wy] |= MM_WALL(dir);

    int nx = wx + dx[dir];
    int ny = wy + dy[dir];
    if (in_bounds(nx, ny)) {
        known[nx][ny] |= MM_WALL((dir + 2) & 3);
    }

    if (has_wall) {
        set_wall(wx, wy, dir);
    }
}

/* Convert the mouse-relative sensor bits (left/front/right) into
 * absolute directions using the current heading. */
static void update_walls(void)
{
    uint8_t s = mm_sense();
    observe_side(x, y, heading,           (s & MM_SEE_FRONT) != 0);
    observe_side(x, y, (heading + 1) & 3, (s & MM_SEE_RIGHT) != 0);
    observe_side(x, y, (heading + 3) & 3, (s & MM_SEE_LEFT)  != 0);
}

/* Sense the current cell only the first time we stand on it (the rest of the
 * solver then works from the stored map). Front, left and right are read now;
 * the side behind us was already stored from the previous cell. */
static bool check_reset(void);

static void explore_cell(void)
{
    check_reset();               /* a reset moves the mouse: never sense with a stale position */
    if (explored[x][y]) {
        return;
    }
    update_walls();
    explored[x][y] = true;
}

/* Heading changes only if the turn really happened. */
static mm_status_t turn(int quarter_turns)
{
    mm_status_t st = mm_turn(quarter_turns);
    if (st == MM_OK) {
        heading = ((heading + quarter_turns) % 4 + 4) % 4;
    }
    return st;
}

/* Position changes only if the move really happened. */
static mm_status_t move_forward(void)
{
    mm_status_t st = mm_forward(1);
    if (st == MM_OK) {
        x += dx[heading];
        y += dy[heading];
    }
    return st;
}

/* Returns the status, so the caller can stop if a turn fails. */
static mm_status_t turn_to(int target_dir)
{
    int diff = (target_dir - heading + 4) % 4;
    if (diff == 1) {
        return turn(1);
    } else if (diff == 2) {
        return turn(2);
    } else if (diff == 3) {
        return turn(-1);
    }
    return MM_OK;
}

static bool check_reset(void)
{
    if (mm_reset_requested()) {
        x = 0;
        y = 0;
        heading = MM_NORTH;
        mm_reset_done();
        mm_log("reset detected: back at (0,0) facing north");
        return true;
    }
    return false;
}

static bool can_pass(int cx, int cy, int dir, bool known_only)
{
    if (walls[cx][cy] & MM_WALL(dir)) {
        return false;
    }
    if (known_only && !(known[cx][cy] & MM_WALL(dir))) {
        return false;
    }
    int nx = cx + dx[dir];
    int ny = cy + dy[dir];
    return in_bounds(nx, ny);
}

static void report_map(void)
{
    mm_map_begin();
    for (int cy = 0; cy < maze.height; cy++) {
        for (int cx = 0; cx < maze.width; cx++) {
            mm_map_cell(cx, cy, walls[cx][cy], known[cx][cy]);
        }
    }
    mm_map_end();
}

/* ======================================================================== */
/* C. FLOOD FILL                                                             */
/*    1. flood_estimate(): breadth-first distances to the target, using the  */
/*       walls known so far (unseen sides count as open)                     */
/*    2. flood_reflood() : queue re-flood when a cell is not 1 + best        */
/*       neighbour (no recursion, fixed memory)                              */
/* ======================================================================== */
static int dist[MM_MAX_SIZE][MM_MAX_SIZE];

/* Current target box; its cells always stay at distance 0. */
static int tgt_x0, tgt_y0, tgt_x1, tgt_y1;

static bool is_target(int cx, int cy)
{
    return in_box(cx, cy, tgt_x0, tgt_y0, tgt_x1, tgt_y1);
}

/* ---- Step 1: start values = breadth-first distances over the known map ----
 * Sides without a recorded wall count as open, so on a fresh maze this gives
 * exactly the Manhattan distance; on a partly explored maze it already avoids
 * the dead ends the mouse knows about. Cells that cannot reach the target
 * stay INF. */
static void flood_estimate(int tx0, int ty0, int tx1, int ty1)
{
    static uint8_t qx[MM_MAX_SIZE * MM_MAX_SIZE];
    static uint8_t qy[MM_MAX_SIZE * MM_MAX_SIZE];
    int head = 0, tail = 0;

    tgt_x0 = tx0; tgt_y0 = ty0; tgt_x1 = tx1; tgt_y1 = ty1;

    for (int cy = 0; cy < maze.height; cy++) {
        for (int cx = 0; cx < maze.width; cx++) {
            dist[cx][cy] = INF;
        }
    }

    for (int cy = ty0; cy <= ty1; cy++) {
        for (int cx = tx0; cx <= tx1; cx++) {
            if (in_bounds(cx, cy)) {
                dist[cx][cy] = 0;
                qx[tail] = (uint8_t)cx;
                qy[tail] = (uint8_t)cy;
                tail++;
            }
        }
    }

    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        for (int d = 0; d < 4; d++) {
            if (!can_pass(cx, cy, d, false)) {
                continue;
            }
            int nx = cx + dx[d], ny = cy + dy[d];
            if (dist[nx][ny] > dist[cx][cy] + 1) {
                dist[nx][ny] = dist[cx][cy] + 1;
                qx[tail] = (uint8_t)nx;
                qy[tail] = (uint8_t)ny;
                tail++;
            }
        }
    }
}

/* ---- Step 2: re-flood with a FIFO queue ------------------------------------ */
#define Q_CAP (MM_MAX_SIZE * MM_MAX_SIZE)

static uint8_t q_x[Q_CAP];
static uint8_t q_y[Q_CAP];
static uint8_t q_in[MM_MAX_SIZE][MM_MAX_SIZE];     /* 1 = cell already queued */
static int q_head, q_count;

/* A cell is queued at most once at a time, so the queue never holds more than
 * width*height entries and cannot overflow. First in, first out means the
 * corrections spread outward in waves, which needs far fewer passes than a
 * last-in-first-out stack. */
static void q_push(int cx, int cy)
{
    if (!in_bounds(cx, cy) || q_in[cx][cy]) {
        return;
    }
    int idx = (q_head + q_count) % Q_CAP;
    q_x[idx] = (uint8_t)cx;
    q_y[idx] = (uint8_t)cy;
    q_count++;
    q_in[cx][cy] = 1;
}

static void q_pop(int *cx, int *cy)
{
    *cx = q_x[q_head];
    *cy = q_y[q_head];
    q_in[*cx][*cy] = 0;
    q_head = (q_head + 1) % Q_CAP;
    q_count--;
}

/* Smallest dist among neighbours we can walk to (no wall known). */
static int min_open_neighbour(int cx, int cy)
{
    int m = INF;
    for (int d = 0; d < 4; d++) {
        if (!can_pass(cx, cy, d, false)) {
            continue;
        }
        int nd = dist[cx + dx[d]][cy + dy[d]];
        if (nd < m) {
            m = nd;
        }
    }
    return m;
}

static void push_open_neighbours(int cx, int cy)
{
    for (int d = 0; d < 4; d++) {
        if (can_pass(cx, cy, d, false)) {
            q_push(cx + dx[d], cy + dy[d]);
        }
    }
}

static void flood_reflood(int sx, int sy)
{
    const int  cap       = maze.width * maze.height;   /* no real path is this long */
    const long max_loops = 50000L;                     /* safety net (never reached in tests) */
    long loops = 0;

    q_head = 0;
    q_count = 0;
    memset(q_in, 0, sizeof(q_in));

    /* The current cell first, then its neighbours: the cell behind a newly
     * found wall needs its old value corrected too. */
    q_push(sx, sy);
    for (int d = 0; d < 4; d++) {
        q_push(sx + dx[d], sy + dy[d]);
    }

    while (q_count > 0) {
        if (++loops > max_loops) {
            /* Never leave dist[][] half updated: rebuild it from the known walls. */
            mm_log("reflood safety stop: rebuilding distances");
            flood_estimate(tgt_x0, tgt_y0, tgt_x1, tgt_y1);
            break;
        }

        int cx, cy;
        q_pop(&cx, &cy);
        if (is_target(cx, cy)) {
            continue;                                  /* target stays 0 */
        }

        int m   = min_open_neighbour(cx, cy);
        int cur = dist[cx][cy];

        /* Sealed-off area: mark it unreachable so the loop always ends. */
        if (m >= INF || m + 1 > cap) {
            if (cur < INF) {
                dist[cx][cy] = INF;
                push_open_neighbours(cx, cy);
            }
            continue;
        }

        if (cur != m + 1) {                            /* not consistent: fix it */
            dist[cx][cy] = m + 1;
            push_open_neighbours(cx, cy);
        }
    }
}

/* Open neighbour with the smallest distance (tie: straight, right, left, back). */
static int choose_best_dir(void)
{
    static const int order[4] = {0, 1, 3, 2};
    int best_dir = -1;
    int best_dist = INF;

    for (int i = 0; i < 4; i++) {
        int dir = (heading + order[i]) & 3;
        if (!can_pass(x, y, dir, false)) {
            continue;
        }
        int nd = dist[x + dx[dir]][y + dy[dir]];
        if (nd < best_dist) {
            best_dist = nd;
            best_dir = dir;
        }
    }
    return best_dir;
}

static void show_distances(void)
{
    char buf[12];
    for (int cy = 0; cy < maze.height; cy++) {
        for (int cx = 0; cx < maze.width; cx++) {
            if (dist[cx][cy] >= INF) {
                mm_label(cx, cy, "?");
            } else {
                snprintf(buf, sizeof(buf), "%d", dist[cx][cy]);
                mm_label(cx, cy, buf);
            }
        }
    }
}

/* ======================================================================== */
/* D. SPEED RUN: route over the known map, then follow it                    */
/* ======================================================================== */
#define SR_INF     1000000L
#define MAX_STATES (MM_MAX_SIZE * MM_MAX_SIZE * 4)   /* cell x heading */

static uint8_t path[MAX_STATES];       /* one absolute direction per step */
static int     path_len;

static long    sd[MAX_STATES];         /* best cost to reach this state   */
static int     sprev[MAX_STATES];      /* state we came from              */
static bool    sdone[MAX_STATES];

static int state_of(int cx, int cy, int h)
{
    return (cx * MM_MAX_SIZE + cy) * 4 + h;
}

/* ---- route: Dijkstra over (cell, heading), known sides only ---------------
 * step cost = 1 + TURN_COST per 90 degrees turned. The cheapest state inside
 * the target box wins. Arrays are fixed size, no heap, no recursion. */
static bool route(int tx0, int ty0, int tx1, int ty1)
{
    for (int i = 0; i < MAX_STATES; i++) {
        sd[i]    = SR_INF;
        sprev[i] = -1;
        sdone[i] = false;
    }
    sd[state_of(x, y, heading)] = 0;

    int goal = -1;
    for (;;) {
        int  u = -1;
        long best = SR_INF;
        for (int i = 0; i < MAX_STATES; i++) {
            if (!sdone[i] && sd[i] < best) {
                best = sd[i];
                u = i;
            }
        }
        if (u < 0) {
            break;                                    /* nothing left to reach */
        }
        sdone[u] = true;

        int h  = u & 3;
        int cx = (u >> 2) / MM_MAX_SIZE;
        int cy = (u >> 2) % MM_MAX_SIZE;
        if (in_box(cx, cy, tx0, ty0, tx1, ty1)) {
            goal = u;
            break;
        }

        for (int d = 0; d < 4; d++) {
            if (!can_pass(cx, cy, d, true)) {
                continue;                             /* wall, or side never seen */
            }
            int diff = (d - h) & 3;
            long turn_cost = (diff == 0) ? 0 : (diff == 2 ? 2L * TURN_COST : (long)TURN_COST);
            int  v  = state_of(cx + dx[d], cy + dy[d], d);
            long nc = sd[u] + 1 + turn_cost;
            if (nc < sd[v]) {
                sd[v]    = nc;
                sprev[v] = u;
            }
        }
    }
    if (goal < 0) {
        return false;
    }

    /* rebuild the path (start -> goal) from the "came from" links */
    int n = 0;
    for (int s = goal; sprev[s] >= 0; s = sprev[s]) {
        n++;
    }
    if (n > MAX_STATES) {
        return false;                                 /* cannot happen; safety */
    }
    path_len = n;
    int s = goal;
    for (int i = n - 1; i >= 0; i--) {
        path[i] = (uint8_t)(s & 3);                   /* direction of the move into s */
        s = sprev[s];
    }
    return true;
}

/* Colour used to paint the cells of the current trip. A cell is painted only
 * AFTER the mouse has really entered it, never in advance. */
static char path_color = MM_COLOR_GREEN;

/* Drive n cells in the current heading with ONE mm_forward() call. */
static bool move_cells(int n)
{
    int x0 = x, y0 = y;
    /* Note for hal_esp32.cpp: the solver assumes a BLOCKED mm_forward(n) left
     * the mouse where it was (this is what mouse_hal.h promises). */
    mm_status_t st = mm_forward(n);
    if (st == MM_OK) {
        for (int k = 1; k <= n; k++) {
            mm_mark(x0 + dx[heading] * k, y0 + dy[heading] * k, path_color);
        }
        x += dx[heading] * n;
        y += dy[heading] * n;
        return true;
    }
    if (st == MM_BLOCKED && n > 1) {
        /* A wall is somewhere ahead and the mouse did not move. Go one cell
         * at a time, so the wall is recorded on the cell that really has it
         * (and not on the cell we are standing on). */
        for (int k = 0; k < n; k++) {
            st = move_forward();
            if (st != MM_OK) {
                break;
            }
            mm_mark(x, y, path_color);
        }
        if (st == MM_OK) {
            return true;
        }
    }
    if (check_reset()) {
        return false;            /* the operator restarted the mouse: our x, y were stale */
    }
    if (st == MM_BLOCKED) {
        /* the map said open but a wall stopped us: learn it */
        observe_side(x, y, heading, true);
        mm_log("speed run blocked at (%d,%d), wall recorded", x, y);
    } else {
        mm_log("speed run fault at (%d,%d)", x, y);
    }
    return false;
}

/* ---- follow: drive the planned path ---------------------------------------- */
static bool follow(void)
{
    int i = 0;
    while (i < path_len) {
        if (check_reset()) {
            mm_log("speed run: reset detected, plan dropped");
            return false;        /* the caller starts again from (0,0) */
        }
        int d = path[i];
        int run = 1;
        if (MERGE_STRAIGHTS) {
            while (i + run < path_len && path[i + run] == d) {
                run++;
            }
        }
        if (turn_to(d) != MM_OK) {
            if (check_reset()) {
                return false;
            }
            mm_log("speed run: turn failed at (%d,%d)", x, y);
            return false;
        }
        if (run == 1) {
            mm_status_t st = move_forward();
            if (st != MM_OK) {
                if (check_reset()) {
                    return false;        /* do not record a wall at a stale cell */
                }
                if (st == MM_BLOCKED) {
                    observe_side(x, y, heading, true);
                }
                mm_log("speed run: move failed at (%d,%d)", x, y);
                return false;
            }
            mm_mark(x, y, path_color);
        } else if (!move_cells(run)) {
            return false;
        }
        i += run;
    }
    return true;
}

static bool speed_run(int tx0, int ty0, int tx1, int ty1, char color)
{
    path_color = color;
    if (!route(tx0, ty0, tx1, ty1)) {
        mm_log("speed run: no fully known path yet");
        return false;
    }
    mm_log("speed run: %d cells", path_len);
    return follow() && in_box(x, y, tx0, ty0, tx1, ty1);
}

/* ======================================================================== */
/* E. SEARCH + MAIN                                                          */
/* ======================================================================== */
/* Explore toward the target box, learning walls on the way.
 * Each step:
 *   explore the cell -> if its value is not 1 + (best neighbour), re-flood
 *   -> move to the best neighbour. */
static bool flood_search(int tx0, int ty0, int tx1, int ty1, char trail_color)
{
    const int max_steps = maze.width * maze.height * 8;

    flood_estimate(tx0, ty0, tx1, ty1);          /* start values (BFS over known map) */

    for (int step = 0; step < max_steps; step++) {
        if (check_reset()) {
            step = 0;                            /* a reset restarts the step budget */
        }

        explore_cell();                          /* sense this cell once */
        if (in_box(x, y, tx0, ty0, tx1, ty1)) {
            return true;
        }
        mm_mark(x, y, trail_color);

        int best  = choose_best_dir();
        int min_d = (best >= 0) ? dist[x + dx[best]][y + dy[best]] : INF;

        if (best < 0 || dist[x][y] != 1 + min_d) {
            flood_reflood(x, y);                 /* values are out of date */
            best = choose_best_dir();
        }
        show_distances();

        if (best < 0 || dist[x][y] >= INF) {
            mm_log("no path from (%d,%d) to target", x, y);
            return false;
        }

        if (turn_to(best) != MM_OK) {
            if (check_reset()) {
                step = 0;
                continue;
            }
            mm_log("turn failed at (%d,%d)", x, y);
            return false;
        }

        mm_status_t st = move_forward();
        if (st == MM_BLOCKED) {
            if (check_reset()) {
                step = 0;                        /* reset: x, y were stale, record nothing */
                continue;
            }
            observe_side(x, y, heading, true);   /* the sensor missed a wall: record it */
            continue;                            /* next round re-floods */
        } else if (st == MM_FAULT) {
            if (check_reset()) {
                step = 0;
                continue;
            }
            mm_log("hardware/sim fault at (%d,%d)", x, y);
            return false;
        }
    }

    mm_log("step limit reached before target");
    return false;
}

/* Visit every cell that has not been explored yet, one after the other.
 * A cell that cannot be reached is skipped. */
static void explore_row_by_row(void)
{
    for (int cx = 0; cx < maze.width; cx++) {
        for (int cy = 0; cy < maze.height; cy++) {
            if (explored[cx][cy]) {
                continue;
            }
            if (!flood_search(cx, cy, cx, cy, MM_COLOR_ORANGE)) {
                mm_log("cell (%d,%d) not reachable, skipped", cx, cy);
            }
        }
    }
}

/* Nearest cell that has not been explored yet and can be reached through
 * sides the mouse has already seen open (breadth-first from the mouse).
 * Cells in skip[][] are ignored. Returns false when nothing is left. */
static bool nearest_unexplored(bool skip[MM_MAX_SIZE][MM_MAX_SIZE], int *fx, int *fy)
{
    static uint8_t qx[MM_MAX_SIZE * MM_MAX_SIZE];
    static uint8_t qy[MM_MAX_SIZE * MM_MAX_SIZE];
    static bool    seen[MM_MAX_SIZE][MM_MAX_SIZE];
    int head = 0, tail = 0;

    memset(seen, 0, sizeof(seen));
    qx[tail] = (uint8_t)x;
    qy[tail] = (uint8_t)y;
    tail++;
    seen[x][y] = true;

    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        if (!explored[cx][cy] && !skip[cx][cy]) {
            *fx = cx;
            *fy = cy;
            return true;
        }
        for (int d = 0; d < 4; d++) {
            if (!can_pass(cx, cy, d, true)) {
                continue;                        /* wall, or side never seen */
            }
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!seen[nx][ny]) {
                seen[nx][ny] = true;
                qx[tail] = (uint8_t)nx;
                qy[tail] = (uint8_t)ny;
                tail++;
            }
        }
    }
    return false;
}

/* Always go to the NEAREST unexplored cell along known paths, sense it, and
 * repeat. This keeps the trips short, so far fewer paths and turns are
 * repeated than when visiting cells row by row. */
static void explore_nearest_first(void)
{
    static bool skip[MM_MAX_SIZE][MM_MAX_SIZE];
    int fx, fy;

    memset(skip, 0, sizeof(skip));
    while (nearest_unexplored(skip, &fx, &fy)) {
        if (!speed_run(fx, fy, fx, fy, MM_COLOR_ORANGE)) {
            /* the known route failed (a wall was learned on the way):
             * try the exploring search, and give up on this cell if it fails */
            if (!flood_search(fx, fy, fx, fy, MM_COLOR_ORANGE)) {
                skip[fx][fy] = true;
                mm_log("cell (%d,%d) not reachable, skipped", fx, fy);
                continue;
            }
        }
        if (check_reset()) {
            continue;                            /* the operator restarted the mouse: plan again from (0,0) */
        }
        explore_cell();                          /* sense the cell we arrived at */
        if (!explored[x][y] || (x != fx || y != fy)) {
            skip[fx][fy] = true;                 /* never loop on one cell */
        }
    }
}

void solver_run(void)
{
    mm_begin("T03");

    map_init();
    mm_log("solver started: maze %dx%d goal (%d,%d)-(%d,%d)",
           maze.width, maze.height,
           maze.goal_x0, maze.goal_y0, maze.goal_x1, maze.goal_y1);

    mm_clear_marks();
    for (int gy = maze.goal_y0; gy <= maze.goal_y1; gy++) {
        for (int gx = maze.goal_x0; gx <= maze.goal_x1; gx++) {
            mm_mark(gx, gy, MM_COLOR_BLUE);
        }
    }

    /* 1) start -> centre */
    if (!flood_search(maze.goal_x0, maze.goal_y0,
                      maze.goal_x1, maze.goal_y1, MM_COLOR_GRAY)) {
        report_map();
        return;
    }
    mm_log("reached goal at (%d,%d) at %lu ms", x, y, (unsigned long)mm_millis());

    /* 2) map the rest of the maze */
    if (EXPLORE_MODE == 1) {
        explore_nearest_first();
    } else if (EXPLORE_MODE == 2) {
        explore_row_by_row();
    }

    /* 3) back to the start: route over the known map, falling back to the
     *    exploring search if no fully known path exists */
    mm_log("Back to starting point...");
    if (!speed_run(0, 0, 0, 0, MM_COLOR_CYAN)) {
        if (!flood_search(0, 0, 0, 0, MM_COLOR_CYAN)) {
            report_map();
            return;
        }
    }
    mm_log("returned to start at %lu ms", (unsigned long)mm_millis());

    /* 4) speed run to the centre */
    mm_log("Going to goal...");
    bool ok = speed_run(maze.goal_x0, maze.goal_y0,
                        maze.goal_x1, maze.goal_y1, MM_COLOR_GREEN);
    if (check_reset()) {
        ok = false;                              /* restarted at the end of the run: drive it again */
    }
    if (!ok) {
        mm_log("speed run not completed, using the exploring search");
        ok = flood_search(maze.goal_x0, maze.goal_y0,
                          maze.goal_x1, maze.goal_y1, MM_COLOR_GREEN);
    }

    if (ok) {
        mm_log("shortest run finished at (%d,%d) at %lu ms",
               x, y, (unsigned long)mm_millis());
    } else {
        mm_log("SPEED RUN FAILED at (%d,%d) at %lu ms",
               x, y, (unsigned long)mm_millis());
    }

    report_map();
}
