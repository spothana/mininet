/* routing_bst.c
 * Binary Search Tree: IP string (key) -> MAC address (value).
 * Models an ARP / forwarding table lookup.
 *
 * Data structures exercised: BST, recursive algorithms, char[], uint8_t[]
 */
#include "mininet.h"

static RouteNode *node_new(const char *ip, const uint8_t mac[6]) {
    RouteNode *n = malloc(sizeof(RouteNode));
    if (!n) return NULL;
    strncpy(n->ip, ip, sizeof(n->ip) - 1);
    n->ip[sizeof(n->ip)-1] = '\0';
    memcpy(n->mac, mac, 6);
    n->left = n->right = NULL;
    return n;
}

RouteNode *bst_insert(RouteNode *root, const char *ip, const uint8_t mac[6]) {
    if (!root) return node_new(ip, mac);
    int cmp = ip_cmp(ip, root->ip);
    if      (cmp < 0) root->left  = bst_insert(root->left,  ip, mac);
    else if (cmp > 0) root->right = bst_insert(root->right, ip, mac);
    else              memcpy(root->mac, mac, 6); /* update on dup key */
    return root;
}

RouteNode *bst_search(RouteNode *root, const char *ip) {
    if (!root) return NULL;
    int cmp = ip_cmp(ip, root->ip);
    if      (cmp < 0) return bst_search(root->left,  ip);
    else if (cmp > 0) return bst_search(root->right, ip);
    else              return root;
}

/* Find in-order successor (leftmost in right subtree) */
static RouteNode *bst_min(RouteNode *root) {
    while (root->left) root = root->left;
    return root;
}

RouteNode *bst_delete(RouteNode *root, const char *ip) {
    if (!root) return NULL;
    int cmp = ip_cmp(ip, root->ip);
    if (cmp < 0) {
        root->left  = bst_delete(root->left,  ip);
    } else if (cmp > 0) {
        root->right = bst_delete(root->right, ip);
    } else {
        /* Node to delete found */
        if (!root->left) {
            RouteNode *tmp = root->right;
            free(root);
            return tmp;
        } else if (!root->right) {
            RouteNode *tmp = root->left;
            free(root);
            return tmp;
        }
        /* Two children: replace with in-order successor */
        RouteNode *succ = bst_min(root->right);
        strncpy(root->ip, succ->ip, sizeof(root->ip)-1);
        memcpy(root->mac, succ->mac, 6);
        root->right = bst_delete(root->right, succ->ip);
    }
    return root;
}

void bst_inorder(const RouteNode *root) {
    if (!root) return;
    bst_inorder(root->left);
    char mac_s[18];
    mac_to_str(root->mac, mac_s);
    printf("  %-15s  ->  %s\n", root->ip, mac_s);
    bst_inorder(root->right);
}

void bst_free(RouteNode *root) {
    if (!root) return;
    bst_free(root->left);
    bst_free(root->right);
    free(root);
}

/* ---- Thread-safe RouteTable wrapper ---- */
void rt_init(RouteTable *rt) {
    rt->root = NULL;
    pthread_mutex_init(&rt->mutex, NULL);
}
void rt_insert(RouteTable *rt, const char *ip, const uint8_t mac[6]) {
    pthread_mutex_lock(&rt->mutex);
    rt->root = bst_insert(rt->root, ip, mac);
    pthread_mutex_unlock(&rt->mutex);
}
RouteNode *rt_search(RouteTable *rt, const char *ip) {
    pthread_mutex_lock(&rt->mutex);
    RouteNode *n = bst_search(rt->root, ip);
    pthread_mutex_unlock(&rt->mutex);
    return n;
}
void rt_delete(RouteTable *rt, const char *ip) {
    pthread_mutex_lock(&rt->mutex);
    rt->root = bst_delete(rt->root, ip);
    pthread_mutex_unlock(&rt->mutex);
}
void rt_print(RouteTable *rt) {
    pthread_mutex_lock(&rt->mutex);
    printf("RouteTable (BST in-order):\n");
    bst_inorder(rt->root);
    pthread_mutex_unlock(&rt->mutex);
}
void rt_free(RouteTable *rt) {
    pthread_mutex_lock(&rt->mutex);
    bst_free(rt->root);
    rt->root = NULL;
    pthread_mutex_unlock(&rt->mutex);
    pthread_mutex_destroy(&rt->mutex);
}
