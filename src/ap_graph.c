/* ap_graph.c
 * Graph of Access Points where edges represent RF interference overlap.
 * Stored as adjacency list (array of fixed-size neighbor arrays).
 * BFS: find shortest hop path for roaming decisions.
 * DFS: detect all APs reachable from a starting AP (coverage mapping).
 *
 * Data structures exercised: graph (adjacency list), BFS queue, DFS stack,
 *                             int arrays, char strings
 */
#include "mininet.h"

/* Simple int queue for BFS (static array, no heap alloc needed) */
typedef struct {
    int data[MAX_APS];
    int front, rear, count;
} IntQueue;

static void iq_init(IntQueue *q)             { q->front=q->rear=q->count=0; }
static void iq_push(IntQueue *q, int v)      { q->data[q->rear++]=v; q->count++; }
static int  iq_pop (IntQueue *q)             { q->count--; return q->data[q->front++]; }
static int  iq_empty(const IntQueue *q)      { return q->count==0; }

/* Simple int stack for DFS */
typedef struct {
    int data[MAX_APS];
    int top;
} IntStack;

static void is_init(IntStack *s)             { s->top=-1; }
static void is_push(IntStack *s, int v)      { s->data[++s->top]=v; }
static int  is_pop (IntStack *s)             { return s->data[s->top--]; }
static int  is_empty(const IntStack *s)      { return s->top<0; }

/* ---- public API ---- */

void graph_init(APGraph *g) {
    memset(g, 0, sizeof(APGraph));
    g->ap_count = 0;
}

/* Returns new AP id (0-based), or -1 if full */
int graph_add_ap(APGraph *g, const char *ssid, Band band,
                 int channel, int tx_power_dbm) {
    if (g->ap_count >= MAX_APS) return -1;
    int id = g->ap_count++;
    AccessPoint *ap = &g->aps[id];
    ap->id = id;
    strncpy(ap->ssid, ssid, 32);
    ap->band          = band;
    ap->channel       = channel;
    ap->tx_power_dbm  = tx_power_dbm;
    ap->neighbor_count = 0;
    return id;
}

/* Add undirected edge between ap_a and ap_b with interference % */
void graph_add_edge(APGraph *g, int ap_a, int ap_b, int interference_pct) {
    AccessPoint *a = &g->aps[ap_a];
    AccessPoint *b = &g->aps[ap_b];
    if (a->neighbor_count < MAX_AP_NEIGHBORS) {
        a->neighbors[a->neighbor_count].neighbor_id     = ap_b;
        a->neighbors[a->neighbor_count].interference_pct = interference_pct;
        a->neighbor_count++;
    }
    if (b->neighbor_count < MAX_AP_NEIGHBORS) {
        b->neighbors[b->neighbor_count].neighbor_id     = ap_a;
        b->neighbors[b->neighbor_count].interference_pct = interference_pct;
        b->neighbor_count++;
    }
}

/* BFS from start_id: prints shortest hop path order */
void graph_bfs(const APGraph *g, int start_id) {
    int visited[MAX_APS] = {0};
    int dist[MAX_APS];
    for (int i = 0; i < MAX_APS; i++) { dist[i]=-1; }

    IntQueue q;
    iq_init(&q);
    visited[start_id] = 1;
    dist[start_id]    = 0;
    iq_push(&q, start_id);

    printf("  BFS from AP[%d] '%s':\n", start_id, g->aps[start_id].ssid);
    while (!iq_empty(&q)) {
        int cur = iq_pop(&q);
        printf("    hop=%d  AP[%d] '%s' ch=%d\n",
               dist[cur], cur, g->aps[cur].ssid, g->aps[cur].channel);
        const AccessPoint *ap = &g->aps[cur];
        for (int i = 0; i < ap->neighbor_count; i++) {
            int nb = ap->neighbors[i].neighbor_id;
            if (!visited[nb]) {
                visited[nb] = 1;
                dist[nb]    = dist[cur] + 1;
                iq_push(&q, nb);
            }
        }
    }
}

/* DFS iterative from start_id */
void graph_dfs(const APGraph *g, int start_id, int visited[]) {
    IntStack stk;
    is_init(&stk);
    is_push(&stk, start_id);
    printf("  DFS from AP[%d] '%s':\n", start_id, g->aps[start_id].ssid);
    while (!is_empty(&stk)) {
        int cur = is_pop(&stk);
        if (visited[cur]) continue;
        visited[cur] = 1;
        printf("    visited AP[%d] '%s'\n", cur, g->aps[cur].ssid);
        const AccessPoint *ap = &g->aps[cur];
        /* push neighbors in reverse so leftmost is processed first */
        for (int i = ap->neighbor_count - 1; i >= 0; i--) {
            int nb = ap->neighbors[i].neighbor_id;
            if (!visited[nb]) is_push(&stk, nb);
        }
    }
}

static const char *band_str(Band b) {
    switch(b){case BAND_2G:return"2.4GHz";case BAND_5G:return"5GHz";
              case BAND_6G:return"6GHz";default:return"?";}
}

void graph_print(const APGraph *g) {
    printf("AP Interference Graph [%d APs]\n", g->ap_count);
    for (int i = 0; i < g->ap_count; i++) {
        const AccessPoint *ap = &g->aps[i];
        printf("  AP[%d] SSID='%-20s' band=%-6s ch=%-3d txPwr=%ddBm\n",
               ap->id, ap->ssid, band_str(ap->band), ap->channel, ap->tx_power_dbm);
        for (int j = 0; j < ap->neighbor_count; j++) {
            printf("         -> AP[%d] interference=%d%%\n",
                   ap->neighbors[j].neighbor_id,
                   ap->neighbors[j].interference_pct);
        }
    }
}
