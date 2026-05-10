/**
 * =============================================================================
 * MODULE DE SECURITE POUR ESSAIM DE MICRO DRONES
 * Detection de la paire de drones la plus proche en 3D
 * =============================================================================
 * Contraintes principales :
 *   - structure imposee : struct Drone avec id, x, y, z ;
 *   - allocation dynamique des drones dans un seul bloc contigu ;
 *   - aucune indexation par crochets ;
 *   - navigation uniquement par pointeurs et decalages ;
 *   - complexite globale : O(n log^2 n).
 *
 * Compilation conseillee :
 *   gcc -O2 -Wall -Wextra -pedantic -o Code_Source_Industriel.c -lm
 * =============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* ============================================================
 * SECTION 1 : CONSTANTES DU SYSTEME
 * ============================================================ */

#define N_DRONES 10000          /* Taille nominale de l essaim              */
#define ESPACE_MAX 5000.0f      /* Limite du volume de vol en metres        */
#define SEUIL_FORCE_BRUTE 16    /* Seuil du cas de base recursif            */
#define VALIDATION_FORCEE 0     /* Mettre 1 pour forcer deux drones proches */

/* ============================================================
 * SECTION 2 : DEFINITION DE LA TOPOLOGIE DE LA DONNEE
 * ============================================================ */

struct Drone {
    int id;      /* Identifiant unique du drone                 */
    float x;     /* Coordonne spatiale X en metres              */
    float y;     /* Coordonne spatiale Y en metres              */
    float z;     /* Coordonne spatiale Z en metres              */
};

/* ============================================================
 * SECTION 3 : STRUCTURE DU RESULTAT
 * ============================================================ */

/**
 * Paire de drones identifiee comme la plus proche.
 * La distance est gardee au carre pour eviter sqrt dans les comparaisons.
 */
typedef struct {
    struct Drone *drone_a;     /* Pointeur vers le premier drone            */
    struct Drone *drone_b;     /* Pointeur vers le second drone             */
    double distance2;          /* Distance euclidienne au carre             */
} PaireCritique;

/* ============================================================
 * SECTION 4 : STRUCTURE DE CELLULE SPATIALE
 * ============================================================ */

/**
 * Cellule utilisee pendant l analyse de la bande centrale.
 * Elle permet de limiter les comparaisons aux voisins proches.
 */
typedef struct {
    struct Drone *drone;       /* Drone place dans la cellule               */
    long long cx;              /* Coordonne entiere de cellule selon X      */
    long long cy;              /* Coordonne entiere de cellule selon Y      */
    long long cz;              /* Coordonne entiere de cellule selon Z      */
} CelluleDrone;

/* ============================================================
 * SECTION 5 : OUTILS DE DISTANCE ET DE MISE A JOUR
 * ============================================================ */

/** Cree une paire vide avec une distance infinie. */
static PaireCritique paire_vide(void)
{
    PaireCritique p;
    p.drone_a = NULL;
    p.drone_b = NULL;
    p.distance2 = DBL_MAX;
    return p;
}

/** Calcule la distance euclidienne au carre entre deux drones. */
static double distance_carree(const struct Drone *a, const struct Drone *b)
{
    double dx = (double)a->x - (double)b->x;
    double dy = (double)a->y - (double)b->y;
    double dz = (double)a->z - (double)b->z;
    return dx * dx + dy * dy + dz * dz;
}

/** Met a jour la meilleure paire si la nouvelle paire est plus proche. */
static void ameliorer_paire(PaireCritique *best, struct Drone *a, struct Drone *b)
{
    if (a == NULL || b == NULL || a == b) {
        return;
    }

    double d2 = distance_carree(a, b);
    if (d2 < best->distance2) {
        best->distance2 = d2;
        best->drone_a = a;
        best->drone_b = b;
    }
}

/* ============================================================
 * SECTION 6 : FONCTIONS DE COMPARAISON POUR LE TRI
 * ============================================================ */

/** Compare deux drones selon X, puis Y, puis Z. */
static int cmp_drone_x(const struct Drone *a, const struct Drone *b)
{
    if (a->x < b->x) {
        return -1;
    }
    if (a->x > b->x) {
        return 1;
    }
    if (a->y < b->y) {
        return -1;
    }
    if (a->y > b->y) {
        return 1;
    }
    if (a->z < b->z) {
        return -1;
    }
    if (a->z > b->z) {
        return 1;
    }
    return 0;
}

/** Compare deux cles de cellules selon X, puis Y, puis Z. */
static int comparer_cle_cellule(long long ax, long long ay, long long az,
                                long long bx, long long by, long long bz)
{
    if (ax < bx) {
        return -1;
    }
    if (ax > bx) {
        return 1;
    }
    if (ay < by) {
        return -1;
    }
    if (ay > by) {
        return 1;
    }
    if (az < bz) {
        return -1;
    }
    if (az > bz) {
        return 1;
    }
    return 0;
}

/** Compare deux objets CelluleDrone. */
static int cmp_cellule_objet(const CelluleDrone *a, const CelluleDrone *b)
{
    return comparer_cle_cellule(a->cx, a->cy, a->cz, b->cx, b->cy, b->cz);
}

/* ============================================================
 * SECTION 7 : TRI FUSION DES POINTEURS DE DRONES
 * ============================================================ */

/**
 * Fusionne deux parties deja triees du tableau de pointeurs.
 * Les drones ne sont pas deplaces : seules leurs adresses sont deplacees.
 */
static void fusionner_drone_ptr(struct Drone **debut, int gauche_n, int droite_n,
                                struct Drone **temp)
{
    struct Drone **gauche = debut;
    struct Drone **droite = debut + gauche_n;
    struct Drone **fin_gauche = droite;
    struct Drone **fin_droite = debut + gauche_n + droite_n;
    struct Drone **sortie = temp;

    while (gauche < fin_gauche && droite < fin_droite) {
        if (cmp_drone_x(*gauche, *droite) <= 0) {
            *sortie = *gauche;
            gauche++;
        } else {
            *sortie = *droite;
            droite++;
        }
        sortie++;
    }

    while (gauche < fin_gauche) {
        *sortie = *gauche;
        gauche++;
        sortie++;
    }

    while (droite < fin_droite) {
        *sortie = *droite;
        droite++;
        sortie++;
    }

    struct Drone **source = temp;
    struct Drone **cible = debut;
    int reste = gauche_n + droite_n;

    while (reste > 0) {
        *cible = *source;
        cible++;
        source++;
        reste--;
    }
}

/** Trie recursivement les pointeurs de drones par coordonnee X. */
static void tri_fusion_drone_ptr(struct Drone **debut, int n, struct Drone **temp)
{
    if (n <= 1) {
        return;
    }

    int milieu = n / 2;
    tri_fusion_drone_ptr(debut, milieu, temp);
    tri_fusion_drone_ptr(debut + milieu, n - milieu, temp);
    fusionner_drone_ptr(debut, milieu, n - milieu, temp);
}

/* ============================================================
 * SECTION 8 : TRI FUSION DES CELLULES SPATIALES
 * ============================================================ */

/** Fusionne deux parties triees du tableau de cellules. */
static void fusionner_cellules(CelluleDrone *debut, int gauche_n, int droite_n,
                               CelluleDrone *temp)
{
    CelluleDrone *gauche = debut;
    CelluleDrone *droite = debut + gauche_n;
    CelluleDrone *fin_gauche = droite;
    CelluleDrone *fin_droite = debut + gauche_n + droite_n;
    CelluleDrone *sortie = temp;

    while (gauche < fin_gauche && droite < fin_droite) {
        if (cmp_cellule_objet(gauche, droite) <= 0) {
            *sortie = *gauche;
            gauche++;
        } else {
            *sortie = *droite;
            droite++;
        }
        sortie++;
    }

    while (gauche < fin_gauche) {
        *sortie = *gauche;
        gauche++;
        sortie++;
    }

    while (droite < fin_droite) {
        *sortie = *droite;
        droite++;
        sortie++;
    }

    CelluleDrone *source = temp;
    CelluleDrone *cible = debut;
    int reste = gauche_n + droite_n;

    while (reste > 0) {
        *cible = *source;
        cible++;
        source++;
        reste--;
    }
}

/** Trie les cellules pour permettre une recherche rapide par cle. */
static void tri_fusion_cellules(CelluleDrone *debut, int n, CelluleDrone *temp)
{
    if (n <= 1) {
        return;
    }

    int milieu = n / 2;
    tri_fusion_cellules(debut, milieu, temp);
    tri_fusion_cellules(debut + milieu, n - milieu, temp);
    fusionner_cellules(debut, milieu, n - milieu, temp);
}

/* ============================================================
 * SECTION 9 : RECHERCHE DANS LES CELLULES
 * ============================================================ */

/** Teste si une cellule possede exactement la cle recherchee. */
static int meme_cellule(const CelluleDrone *item,
                        long long cx, long long cy, long long cz)
{
    return item->cx == cx && item->cy == cy && item->cz == cz;
}

/** Recherche dichotomique de la premiere cellule de cle donnee. */
static CelluleDrone *borne_inferieure_cellule(CelluleDrone *debut, int n,
                                              long long cx, long long cy, long long cz)
{
    CelluleDrone *bas = debut;
    CelluleDrone *haut = debut + n;

    while (bas < haut) {
        CelluleDrone *milieu = bas + (haut - bas) / 2;
        int ordre = comparer_cle_cellule(milieu->cx, milieu->cy, milieu->cz,
                                         cx, cy, cz);
        if (ordre < 0) {
            bas = milieu + 1;
        } else {
            haut = milieu;
        }
    }

    return bas;
}

/* ============================================================
 * SECTION 10 : FORCE BRUTE DU CAS DE BASE
 * ============================================================ */

/**
 * Compare toutes les paires dans un petit sous probleme.
 * Le seuil etant constant, cette partie reste controlee.
 */
static PaireCritique force_brute(struct Drone **tab, int n)
{
    PaireCritique best = paire_vide();
    struct Drone **pi = tab;
    int i;

    for (i = 0; i < n - 1; i++, pi++) {
        struct Drone **pj = pi + 1;
        int j;

        for (j = i + 1; j < n; j++, pj++) {
            ameliorer_paire(&best, *pi, *pj);
        }
    }

    return best;
}

/* ============================================================
 * SECTION 11 : CONSTRUCTION DE LA GRILLE SPATIALE
 * ============================================================ */

/** Convertit une coordonnee reelle en indice entier de cellule. */
static long long indice_cellule(double coordonnee, double taille_cellule)
{
    return (long long)floor(coordonnee / taille_cellule);
}

/* ============================================================
 * SECTION 12 : COMPARAISON DES CELLULES VOISINES
 * ============================================================ */

/**
 * Pour un drone de la bande, compare seulement avec les drones
 * situes dans les cellules voisines.
 */
static void traiter_cellules_voisines(CelluleDrone *cellules, int n_cellules,
                                      CelluleDrone *courant, PaireCritique *best)
{
    long long dx;
    long long dy;
    long long dz;
    CelluleDrone *fin = cellules + n_cellules;

    for (dx = -2; dx <= 2; dx++) {
        for (dy = -2; dy <= 2; dy++) {
            for (dz = -2; dz <= 2; dz++) {
                long long cible_x = courant->cx + dx;
                long long cible_y = courant->cy + dy;
                long long cible_z = courant->cz + dz;
                CelluleDrone *it = borne_inferieure_cellule(cellules, n_cellules,
                                                            cible_x, cible_y, cible_z);

                while (it < fin && meme_cellule(it, cible_x, cible_y, cible_z)) {
                    ameliorer_paire(best, courant->drone, it->drone);
                    it++;
                }
            }
        }
    }
}

/* ============================================================
 * SECTION 13 : TRAITEMENT DE LA BANDE CENTRALE EN 3D
 * ============================================================ */

/**
 * Analyse les drones proches du plan median.
 * Cette phase verifie les paires qui peuvent traverser la separation.
 */
static void analyser_bande_3d(struct Drone **tab, int n,
                              double x_median, PaireCritique *best,
                              CelluleDrone *cellules,
                              CelluleDrone *temp_cellules)
{
    if (best->distance2 <= 0.0 || best->distance2 == DBL_MAX) {
        return;
    }

    double delta = sqrt(best->distance2);
    double taille_cellule = delta / 2.0;

    if (taille_cellule <= 0.0 || !isfinite(taille_cellule)) {
        return;
    }

    int n_cellules = 0;
    struct Drone **scan = tab;
    struct Drone **fin_tab = tab + n;

    while (scan < fin_tab) {
        struct Drone *d = *scan;
        double ecart_x = fabs((double)d->x - x_median);

        if (ecart_x < delta) {
            CelluleDrone *case_courante = cellules + n_cellules;
            case_courante->drone = d;
            case_courante->cx = indice_cellule((double)d->x, taille_cellule);
            case_courante->cy = indice_cellule((double)d->y, taille_cellule);
            case_courante->cz = indice_cellule((double)d->z, taille_cellule);
            n_cellules++;
        }

        scan++;
    }

    if (n_cellules < 2) {
        return;
    }

    tri_fusion_cellules(cellules, n_cellules, temp_cellules);

    CelluleDrone *courant = cellules;
    CelluleDrone *fin = cellules + n_cellules;

    while (courant < fin) {
        traiter_cellules_voisines(cellules, n_cellules, courant, best);
        courant++;
    }
}

/* ============================================================
 * SECTION 14 : COEUR RECURSIF DIVISER POUR REGNER
 * ============================================================ */

/** Retourne la meilleure des deux paires candidates. */
static PaireCritique meilleure_paire(PaireCritique a, PaireCritique b)
{
    if (a.distance2 <= b.distance2) {
        return a;
    }
    return b;
}

/**
 * Coupe le probleme en deux, resout a gauche et a droite,
 * puis combine avec la bande centrale.
 */
static PaireCritique closest_pair_rec(struct Drone **tab, int n,
                                      CelluleDrone *cellules, CelluleDrone *temp_cellules)
{
    if (n < 2) {
        return paire_vide();
    }

    if (n <= SEUIL_FORCE_BRUTE) {
        return force_brute(tab, n);
    }

    int milieu = n / 2;
    double x_median = (double)(*(tab + milieu))->x;

    PaireCritique gauche = closest_pair_rec(tab, milieu, cellules, temp_cellules);
    PaireCritique droite = closest_pair_rec(tab + milieu, n - milieu, cellules, temp_cellules);
    PaireCritique best = meilleure_paire(gauche, droite);

    analyser_bande_3d(tab, n, x_median, &best, cellules, temp_cellules);

    return best;
}

/* ============================================================
 * SECTION 15 : INITIALISATION ET TABLEAU D INDIRECTION
 * ============================================================ */

/** Remplit le bloc contigu de drones avec des positions de simulation. */
static int initialiser_essaim(struct Drone *essaim, int n)
{
    struct Drone *p = essaim;
    int id = 0;

    while (id < n) {
        p->id = id;
        p->x = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p->y = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p->z = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        p++;
        id++;
    }

#if VALIDATION_FORCEE
    essaim->x = 1000.0f;
    essaim->y = 2000.0f;
    essaim->z = 500.0f;
    (essaim + 1)->x = 1000.1f;
    (essaim + 1)->y = 2000.1f;
    (essaim + 1)->z = 500.1f;
#endif

    return 1;
}

/** Construit un tableau de pointeurs vers les drones du bloc contigu. */
static int construire_index(struct Drone *essaim, struct Drone **index_ptr, int n)
{
    struct Drone *source = essaim;
    struct Drone **cible = index_ptr;
    int reste = n;

    while (reste > 0) {
        *cible = source;
        source++;
        cible++;
        reste--;
    }

    return 1;
}

/* ============================================================
 * SECTION 16 : PROGRAMME PRINCIPAL
 * ============================================================ */

/**
 * Orchestre l'allocation, l'initialisation, le tri, la recherche,
 * l'affichage du resultat et la liberation de la memoire.
 */
int main(void)
{
    int n = N_DRONES;
    srand((unsigned int)time(NULL));

    struct Drone *essaim = (struct Drone *)malloc((size_t)n * sizeof *essaim);
    if (essaim == NULL) {
        fprintf(stderr, "Erreur critique : allocation impossible pour les drones.\n");
        return EXIT_FAILURE;
    }

    struct Drone **index_ptr = (struct Drone **)malloc((size_t)n * sizeof *index_ptr);
    if (index_ptr == NULL) {
        fprintf(stderr, "Erreur critique : allocation impossible pour l index.\n");
        free(essaim);
        return EXIT_FAILURE;
    }

    CelluleDrone *cellules = (CelluleDrone *)malloc((size_t)n * sizeof *cellules);
    if (cellules == NULL) {
        fprintf(stderr, "Erreur critique : allocation impossible pour le buffer geometrique.\n");
        free(index_ptr);
        free(essaim);
        return EXIT_FAILURE;
    }

    CelluleDrone *temp_cellules = (CelluleDrone *)malloc((size_t)n * sizeof *temp_cellules);
    if (temp_cellules == NULL) {
        fprintf(stderr, "Erreur critique : allocation impossible pour le tri geometrique.\n");
        free(cellules);
        free(index_ptr);
        free(essaim);
        return EXIT_FAILURE;
    }

    struct Drone **temp_index = (struct Drone **)malloc((size_t)n * sizeof *temp_index);
    if (temp_index == NULL) {
        fprintf(stderr, "Erreur critique : allocation impossible pour le tri initial.\n");
        free(temp_cellules);
        free(cellules);
        free(index_ptr);
        free(essaim);
        return EXIT_FAILURE;
    }

    initialiser_essaim(essaim, n);
    construire_index(essaim, index_ptr, n);

    tri_fusion_drone_ptr(index_ptr, n, temp_index);

    printf("SDPC : detection de la paire critique pour %d drones.\n", n);
    PaireCritique best = closest_pair_rec(index_ptr, n, cellules, temp_cellules);

    if (best.drone_a != NULL && best.drone_b != NULL) {
        double distance = sqrt(best.distance2);

        printf("\nALERTE SECURITE : paire critique identifiee.\n");
        printf("Drone A : id=%d, position=(%.3f, %.3f, %.3f).\n",
               best.drone_a->id, best.drone_a->x, best.drone_a->y, best.drone_a->z);
        printf("Drone B : id=%d, position=(%.3f, %.3f, %.3f).\n",
               best.drone_b->id, best.drone_b->x, best.drone_b->y, best.drone_b->z);
        printf("Distance minimale : %.9f metres.\n", distance);
        printf("Systeme : manoeuvre d evitement a declencher.\n");
    } else {
        printf("Aucune paire disponible : il faut au moins deux drones.\n");
    }

    free(temp_index);
    free(temp_cellules);
    free(cellules);
    free(index_ptr);
    free(essaim);

    return EXIT_SUCCESS;
}
