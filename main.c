
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

// === DEFINES ===
#define MAX_STOPS 1200
#define MAX_ROUTES 600
#define MAX_PER_ROUTE 120
#define MAX_NAME 120
#define MAX_BUS_NAME 64
#define HSIZE 1021
#define MAX_PQ 20000
#define TRANSFER_PENALTY 30.0

// === STRUCTS ===
typedef struct {
    char name[MAX_NAME];
    double lat, lon;
    char routes[16][MAX_BUS_NAME];
    int n_routes;
} BusStop;

typedef struct {
    char name[MAX_BUS_NAME];
    BusStop *stops[MAX_PER_ROUTE];
    int n_stops;
} Route;

typedef struct Node {
    BusStop *stop;
    char route[MAX_BUS_NAME];
    double cost;
    int transfers;
    int parent_idx;
} Node;

typedef struct Entry {
    char *key;
    double cost;
    struct Entry *next;
} Entry;

static BusStop stops[MAX_STOPS];
static int n_stops = 0;
static Route routes[MAX_ROUTES];
static int n_routes = 0;
static Entry *buckets[HSIZE] = {0};
static BusStop *goal;

BusStop *find_stop(const char *name);
void process_bus(const char *bus_name, int seq_list[], char stop_names[][120], int cnt);
void load_stops(const char *file);
void load_routes(const char *file);
double distance(BusStop *a, BusStop *b);
void find_route(BusStop *start, BusStop *goal);
unsigned int myhash(const char *s);
void hm_put(const char *key, double cost);
double hm_get(const char *key);
void hm_clear(void);
void pq_add(Node n, Node *pq, int *pq_n);
Node pq_pop(Node *pq, int *pq_n);
char *make_key(char *buf, const char *r, BusStop *s);

unsigned int myhash(const char *s) {
    unsigned int h = 5381;
    int c;
    while ((c = *s++)) h = ((h << 5) + h) + c;
    return h % HSIZE;
}

void hm_put(const char *key, double cost) {
    unsigned int idx = myhash(key);
    Entry *e = buckets[idx];
    while (e) {
        if (strcmp(e->key, key) == 0) { e->cost = cost; return; }
        e = e->next;
    }
    e = malloc(sizeof(Entry));
    e->key = strdup(key);
    e->cost = cost;
    e->next = buckets[idx];
    buckets[idx] = e;
}

double hm_get(const char *key) {
    unsigned int idx = myhash(key);
    Entry *e = buckets[idx];
    while (e) {
        if (strcmp(e->key, key) == 0) return e->cost;
        e = e->next;
    }
    return 1e9;
}

void hm_clear(void) {
    for (int i = 0; i < HSIZE; i++) {
        Entry *e = buckets[i];
        while (e) {
            Entry *t = e;
            e = e->next;
            free(t->key);
            free(t);
        }
        buckets[i] = NULL;
    }
}

double distance(BusStop *a, BusStop *b) {
    double R = 6371.0;
    double dlat = (b->lat - a->lat) * M_PI / 180.0;
    double dlon = (b->lon - a->lon) * M_PI / 180.0;
    double la = a->lat * M_PI / 180.0;
    double lb = b->lat * M_PI / 180.0;
    double x = sin(dlat/2)*sin(dlat/2) + cos(la)*cos(lb)*sin(dlon/2)*sin(dlon/2);
    double c = 2 * atan2(sqrt(x), sqrt(1-x));
    return R * c;
}

// === PRIORITY QUEUE ===
void pq_add(Node n, Node *pq, int *pq_n) {
    if (*pq_n < MAX_PQ) pq[(*pq_n)++] = n;
}

Node pq_pop(Node *pq, int *pq_n) {
    int best = 0;
    double best_f = 1e9;
    for (int i = 0; i < *pq_n; i++) {
        double f = pq[i].cost + distance(pq[i].stop, goal);
        if (f < best_f) { best_f = f; best = i; }
    }
    Node res = pq[best];
    pq[best] = pq[--(*pq_n)];
    return res;
}

char *make_key(char *buf, const char *r, BusStop *s) {
    snprintf(buf, 256, "%s|%s", r, s->name);
    return buf;
}

// === FIND STOP ===
BusStop *find_stop(const char *name) {
    if (!name || name[0] == '\0') return NULL;

    for (int i = 0; i < n_stops; i++) {
        if (strcasecmp(stops[i].name, name) == 0) return &stops[i];
    }

    char upper[120];
    strcpy(upper, name);
    for (int i = 0; upper[i]; i++) upper[i] = toupper(upper[i]);

    for (int i = 0; i < n_stops; i++) {
        char s_upper[120];
        strcpy(s_upper, stops[i].name);
        for (int j = 0; s_upper[j]; j++) s_upper[j] = toupper(s_upper[j]);
        if (strstr(s_upper, upper) != NULL) return &stops[i];
    }
    return NULL;
}

// === PROCESS BUS ===
void process_bus(const char *bus_name, int seq_list[], char stop_names[][120], int cnt) {
    if (n_routes >= MAX_ROUTES) return;

    // Sort by sequence
    for (int i = 0; i < cnt - 1; i++) {
        for (int j = i + 1; j < cnt; j++) {
            if (seq_list[i] > seq_list[j]) {
                int t = seq_list[i]; seq_list[i] = seq_list[j]; seq_list[j] = t;
                char temp[120];
                strcpy(temp, stop_names[i]);
                strcpy(stop_names[i], stop_names[j]);
                strcpy(stop_names[j], temp);
            }
        }
    }

    strcpy(routes[n_routes].name, bus_name);
    int valid = 0;

    for (int i = 0; i < cnt; i++) {
        BusStop *s = find_stop(stop_names[i]);
        if (!s) continue;

        routes[n_routes].stops[valid++] = s;

        int has_bus = 0;
        for (int r = 0; r < s->n_routes; r++) {
            if (strcmp(s->routes[r], bus_name) == 0) {
                has_bus = 1;
                break;
            }
        }
        if (!has_bus && s->n_routes < 16) {
            strcpy(s->routes[s->n_routes++], bus_name);
        }
    }
    routes[n_routes].n_stops = valid;
    n_routes++;
}

// === LOAD STOPS ===
void load_stops(const char *file) {
    FILE *f = fopen(file, "r");
    if (!f) { printf("ERROR: Cannot open %s\n", file); return; }

    char line[1024];
    fgets(line, sizeof line, f); // skip header

    while (fgets(line, sizeof line, f)) {
        char *p = line;
        char raw_name[120] = "";
        char lat_str[20] = "", lon_str[20] = "";

        // Parse name
        int i = 0, in_quotes = 0;
        if (*p == '"') { in_quotes = 1; p++; }
        while (*p && ((in_quotes && *p != '"') || (!in_quotes && *p != ','))) {
            if (i < 119) raw_name[i++] = *p;
            p++;
        }
        raw_name[i] = '\0';
        if (in_quotes && *p == '"') p++;
        if (*p == ',') p++;

        // Parse lat
        i = 0;
        while (*p && *p != ',' && i < 19) lat_str[i++] = *p++;
        lat_str[i] = '\0';
        if (*p == ',') p++;

        // Parse lon
        i = 0;
        while (*p && *p != '\n' && *p != '\r' && i < 19) lon_str[i++] = *p++;
        lon_str[i] = '\0';

        double lat = atof(lat_str);
        double lon = atof(lon_str);
        char *name = raw_name;
        name[strcspn(name, "\r\n")] = '\0';

        if (n_stops < MAX_STOPS) {
            strncpy(stops[n_stops].name, name, MAX_NAME - 1);
            stops[n_stops].lat = lat;
            stops[n_stops].lon = lon;
            stops[n_stops].n_routes = 0;
            n_stops++;
        }
    }
    fclose(f);
    printf("Loaded %d stops\n", n_stops);
}

// === LOAD ROUTES ===
void load_routes(const char *file) {
    FILE *f = fopen(file, "r");
    if (!f) { printf("ERROR: Cannot open %s\n", file); return; }

    char line[1024];
    fgets(line, sizeof line, f); // skip header

    char cur_bus[MAX_BUS_NAME] = "";
    int seq_list[MAX_PER_ROUTE];
    char stop_name_list[MAX_PER_ROUTE][120];
    int cnt = 0;

    while (fgets(line, sizeof line, f)) {
        char *p = line;
        char bus[MAX_BUS_NAME] = "", seq_str[12] = "";
        char stop_name[120] = "";

        // Parse bus name
        int i = 0;
        while (*p && *p != ',' && i < MAX_BUS_NAME-1) bus[i++] = *p++;
        bus[i] = '\0';
        if (*p == ',') p++;

        // Parse seq
        i = 0;
        while (*p && *p != ',' && i < 11) seq_str[i++] = *p++;
        seq_str[i] = '\0';
        if (*p == ',') p++;

        // Parse stop name
        i = 0;
        while (*p && *p != '\n' && *p != '\r') {
            if (i < 119) stop_name[i++] = *p;
            p++;
        }
        stop_name[i] = '\0';
        stop_name[strcspn(stop_name, "\r\n")] = '\0';

        if (strcmp(cur_bus, bus) != 0 && cnt > 0) {
            process_bus(cur_bus, seq_list, stop_name_list, cnt);
            cnt = 0;
        }

        strcpy(cur_bus, bus);
        seq_list[cnt] = atoi(seq_str);
        strcpy(stop_name_list[cnt], stop_name);
        cnt++;
    }

    if (cnt > 0) {
        process_bus(cur_bus, seq_list, stop_name_list, cnt);
    }

    fclose(f);
    printf("Loaded %d routes\n", n_routes);
}

// === FIND ROUTE (OPTIMAL + CORRECT TRANSFERS) ===
void find_route(BusStop *start, BusStop *g) {
    goal = g;
    hm_clear();

    Node *pq = malloc(MAX_PQ * sizeof(Node));
    if (!pq) { printf("ERROR: Out of memory!\n"); return; }
    int pq_n = 0;

    typedef struct {
        char route[MAX_BUS_NAME];
        BusStop *stop;
        double cost;
        int transfers;
        int parent_idx;
    } PathNode;

    PathNode *came_from = malloc(20000 * sizeof(PathNode));
    if (!came_from) { free(pq); printf("ERROR: Out of memory!\n"); return; }
    int path_count = 0;
    int goal_idx = -1;

    // Start nodes
    for (int i = 0; i < start->n_routes; i++) {
        char key[256];
        make_key(key, start->routes[i], start);
        hm_put(key, 0.0);

        Node n = { .stop = start, .cost = 0.0, .transfers = 0 };
        strncpy(n.route, start->routes[i], MAX_BUS_NAME - 1);
        n.route[MAX_BUS_NAME - 1] = '\0';
        n.parent_idx = -1;
        pq_add(n, pq, &pq_n);

        came_from[path_count] = (PathNode){
            .stop = start, .cost = 0.0, .transfers = 0, .parent_idx = -1
        };
        strncpy(came_from[path_count].route, start->routes[i], MAX_BUS_NAME - 1);
        came_from[path_count].route[MAX_BUS_NAME - 1] = '\0';
        path_count++;
    }

    while (pq_n > 0 && path_count < 20000) {
        Node cur = pq_pop(pq, &pq_n);

        if (cur.stop == goal) {
            goal_idx = path_count;
            came_from[path_count] = (PathNode){
                .stop = cur.stop, .cost = cur.cost, .transfers = cur.transfers, .parent_idx = cur.parent_idx
            };
            strncpy(came_from[path_count].route, cur.route, MAX_BUS_NAME - 1);
            came_from[path_count].route[MAX_BUS_NAME - 1] = '\0';
            path_count++;
            break;
        }

        Route *rt = NULL;
        for (int r = 0; r < n_routes; r++) {
            if (strcmp(routes[r].name, cur.route) == 0) {
                rt = &routes[r];
                break;
            }
        }
        if (!rt) continue;

        int idx = -1;
        for (int i = 0; i < rt->n_stops; i++) {
            if (rt->stops[i] == cur.stop) { idx = i; break; }
        }

        // Same bus
        int dirs[] = {idx-1, idx+1};
        for (int d = 0; d < 2; d++) {
            int ni = dirs[d];
            if (ni < 0 || ni >= rt->n_stops) continue;
            BusStop *next = rt->stops[ni];
            double d = distance(cur.stop, next);
            double new_cost = cur.cost + d;

            char key[256];
            make_key(key, cur.route, next);
            if (new_cost < hm_get(key)) {
                hm_put(key, new_cost);
                Node n = { .stop = next, .cost = new_cost, .transfers = cur.transfers };
                strncpy(n.route, cur.route, MAX_BUS_NAME - 1);
                n.route[MAX_BUS_NAME - 1] = '\0';
                n.parent_idx = path_count;
                pq_add(n, pq, &pq_n);
            }
        }

        // Transfer
        for (int i = 0; i < cur.stop->n_routes; i++) {
            if (strcmp(cur.stop->routes[i], cur.route) == 0) continue;
            char key[256];
            make_key(key, cur.stop->routes[i], cur.stop);
            double new_cost = cur.cost + TRANSFER_PENALTY;
            if (new_cost < hm_get(key)) {
                hm_put(key, new_cost);
                Node n = { .stop = cur.stop, .cost = new_cost, .transfers = cur.transfers + 1 };
                strncpy(n.route, cur.stop->routes[i], MAX_BUS_NAME - 1);
                n.route[MAX_BUS_NAME - 1] = '\0';
                n.parent_idx = path_count;
                pq_add(n, pq, &pq_n);
            }
        }

        came_from[path_count] = (PathNode){
            .stop = cur.stop, .cost = cur.cost, .transfers = cur.transfers, .parent_idx = cur.parent_idx
        };
        strncpy(came_from[path_count].route, cur.route, MAX_BUS_NAME - 1);
        came_from[path_count].route[MAX_BUS_NAME - 1] = '\0';
        path_count++;
    }

    if (goal_idx == -1) {
        printf("\nNo route found!\n");
        free(pq); free(came_from);
        return;
    }

    // Reconstruct
    typedef struct { BusStop *s; char bus[MAX_BUS_NAME]; } Step;
    Step path[5000];
    int plen = 0;
    int idx = goal_idx - 1;

    while (idx >= 0) {
        path[plen].s = came_from[idx].stop;
        strncpy(path[plen].bus, came_from[idx].route, MAX_BUS_NAME - 1);
        path[plen].bus[MAX_BUS_NAME - 1] = '\0';
        plen++;
        idx = came_from[idx].parent_idx;
    }

    // Reverse
    for (int i = 0; i < plen / 2; i++) {
        Step t = path[i];
        path[i] = path[plen-1-i];
        path[plen-1-i] = t;
    }

    // Print
    printf("\n=== ROUTE FOUND ===\n");
    printf("Transfers: %d\n", came_from[goal_idx-1].transfers);

    int i = 0;
    double total = 0.0;
    int leg_num = 1;

    while (i < plen) {
        char cur_bus[MAX_BUS_NAME];
        strncpy(cur_bus, path[i].bus, MAX_BUS_NAME - 1);
        cur_bus[MAX_BUS_NAME - 1] = '\0';

        printf("\nLEG %d  BUS %s\n", leg_num++, cur_bus);

        int leg_start = i;
        while (i < plen && strcmp(path[i].bus, cur_bus) == 0) {
            printf("  - %s\n", path[i].s->name);
            i++;
        }

        for (int j = leg_start; j < i - 1; j++) {
            total += distance(path[j].s, path[j + 1].s);
        }
    }

    printf("\nTotal distance approximately %.2f km\n", total);

    free(pq);
    free(came_from);
}

int main(void) {
    printf("Loading data...\n");
    load_stops("Masterfile_mumbai_bus_stops.csv");
    load_routes("(TEMPERARY)bus_routes_fully_corrected.csv");

    while (1) {
        printf("\n--- Mumbai Bus Finder ---\n");
        printf("Enter START stop: ");
        char a[120];
        if (!fgets(a, sizeof a, stdin)) break;
        a[strcspn(a, "\n")] = '\0';
        BusStop *s1 = find_stop(a);
        if (!s1) { printf("Stop not found!\n"); continue; }

        printf("Enter END stop: ");
        char b[120];
        if (!fgets(b, sizeof b, stdin)) break;
        b[strcspn(b, "\n")] = '\0';
        BusStop *s2 = find_stop(b);
        if (!s2) { printf("Stop not found!\n"); continue; }
        if (s1 == s2) { printf("Same stop!\n"); continue; }

        find_route(s1, s2);

        printf("\nAgain? (y/n): ");
        char c;
        scanf(" %c", &c);
        if (c != 'y' && c != 'Y') break;
        while (getchar() != '\n');
    }
    return 0;
}
