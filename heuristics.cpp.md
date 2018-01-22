```c++
/*
Copyright 2018 Pierre-Edouard Portier
peportier.me

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
)
*/

```
# Plus court chemin en présence d'information contextuelle

Répertoire GitHub correspondant à cet article : https://github.com/peportier/ia04-heuristics

Comment améliorer la recherche d'un plus court chemin entre un nœud source $s$ et un nœud cible $t$ quand nous possédons pour tout nœud $u$ du graphe une *estimation* $h(u)$ de la distance séparant $u$ de la cible ?

## La notion de fonction heuristique

La fonction $h$ est appelée *fonction heuristique*.

*Déf.* Une heuristique $h$ est *admissible* si pour tout nœud $u$, $h(u)$ est une borne inférieure de la plus courte distance séparant $u$ de la cible $t$, i.e. $h(u) \leq \delta(u,t)$ (avec $\delta(u,t)$ désignant la longueur d'un chemin optimal entre $u$ et $t$ ).

*Déf.* Une heuristique $h$ est *consistante* si pour toute arête $e = (u,v)$ nous avons $h(u) \leq h(v) + w(u,v)$ (avec $w(u,v)$ le poids de l'arête $(u,v)$).

*Déf.* Soient $(u_0,\dots,u_k)$ un chemin et $g(u_i)$ le coût du chemin $(u_0,\dots,u_i)$. Nous posons $f(u_i) = g(u_i) + h(u_i)$.  Une heuristique $h$ est *monotone* si pour tout $j>i, \; 0 \leq i, \; j \leq k$ nous avons $f(u_j) \geq f(u_i)$. C'est-à-dire que l'estimation du poids total d'un chemin ne décroît pas lors du passage d'un nœud à ses successeurs.

Nous remarquons que consistance et monotonicité sont deux propriétés équivalentes.  En effet, pour deux nœuds adjacents $u_{i-1}$ et $u_i$ sur un chemin $(u_0,\dots,u_k)$, nous avons :
$$
\begin{aligned}
& f(u_i) \\
= \quad &\{\text{Déf. de } f\} \\
& g(u_i) + h(u_i) \\
= \quad &\{\text{Déf. du coût d'un chemin}\} \\
& g(u_{i-1}) + w(u_{i-1}, u_i) + h(u_i) \\
\geq \quad &\{\text{Déf. de la consistance}\} \\
& g(u_{i-1}) + h(u_{i-1}) \\
= \quad &\{\text{Déf. de } f\} \\
& f(u_{i-1})
\end{aligned}
$$
L'autre implication (viz. monotonicité implique consistance) est triviale.

Aussi, une heuristique consistante est admissible (la réciproque n'est pas vraie). En effet, si $h$ est consistante, pour toute arête $(u,v)$ nous avons $h(u) - h(v) \leq w(u,v)$. Soit un chemin quelconque $p = (v_0 = u,\dots,v_k = t)$, nous avons :
$$
\begin{aligned}
& w(p) \\
= \quad &\{\text{Déf. du poids d'un chemin}\} \\
& \sum_{i=0}^{k-1} w(v_i, v_{i+1}) \\
\geq \quad &\{\text{Consistance de } h\} \\
& \sum_{i=0}^{k-1} h(v_i) - h(v_{i+1}) \\
= \quad &\{\text{Arithmétique}\} \\
& h(u) - h(t) \\
= \quad &\{t \text{ est la cible}\} \\
& h(u)
\end{aligned}
$$
En particulier, dans le cas d'un plus court chemin : $h(u) \leq \delta(u,t)$.

## L'algorithme A*

L'algorithme A* est un exemple d'amélioration de l'algorithme de Dijkstra grâce à l'utilisation d'une fonction heuristique. A* associe à un nœud $u$ du graphe la valeur :
$$
f(u) = g(u) + h(u)
$$
où $g(u)$ est le poids optimal actuellement connu pour un chemin menant de $s$ à $u$, et $h(u)$ est une heuristique admissible (i.e., une borne inférieure de la distance séparant $u$ de la cible $t$).

Nous remarquons que pour un nœud $u$ gris minimal (au sens de $f(u)$ ), nous avons pour tout successeur $v$ de $u$ :
$$
\begin{aligned}
& f(v) \\
= \quad &\{\text{Déf. de } f\} \\
& g(v) + h(v) \\
= \quad &\{\text{si on devait mettre à jour le poids de } v\} \\
& g(u) + w(u,v) + h(v) \\
= \quad &\{\text{Arithmétique}\} \\
& g(u) + h(u) + w(u,v) - h(u) + h(v) \\
= \quad &\{\text{Déf. de } f\} \\
& f(u) + w(u,v) - h(u) + h(v)
\end{aligned}
$$
Ainsi, l'algorithme A* correspond exactement à l'algorithme de Dijkstra avec la repondération suivante :
$$
\hat{w}(u,v) = w(u,v) - h(u) + h(v)
$$
Si l'heuristique $h$ est monotone alors tous les poids restent positifs et la preuve de correction de Dijkstra s'applique !

De plus, nous allons montrer que, dans le cas d'une heuristique monotone, il n'existe pas d'algorithme $A$ qui trouve la solution optimale en explorant moins de nœuds que $A^*$. Soit $f^* = \delta(s,t)$ le coût de la solution optimale. Nous montrons que tout algorithme optimal (i.e., qui trouvera toujours la solution optimale) doit visiter tous les nœuds $u$ pour lesquels $\delta(s,u) < f^*$.

Travaillons par l'absurde en supposant qu'il existe un algorithme $A$ qui trouve une solution optimale $p_t$ (i.e., avec $w(p_t) = f^*$ ) en ne visitant pas un nœud $u$ pour lequel $\delta(s,u) < f^*$. Montrons qu'il peut alors exister un autre chemin $q$ avec $w(q) < f^*$ qui n'est pas trouvé par A. Soit $q_u$ le chemin tel que $w(q_u) = \delta(s,u)$. Mettons qu'il existe une arête $(u,t)$ de poids $0$ (i.e., $w(u,t) = 0$ ). Puisque le voisinage de $u$ n'a pas été exploré par $A$, ce dernier ne peut pas savoir que l'arête $(u,t)$ existe. Soit le chemin $q = (q_u,t)$. Nous avons : $w(q) = w(q_u) + w(u,t) = \delta(s,u) < f^*$.

## Comparer des heuristiques

Plus une heuristique $h(u)$ est proche de la valeur optimale $\delta(u,t)$ plus elle est *informée*.

Plus une heuristique est informée, mieux elle minimise la taille de l'espace exploré, mais en général elle est également plus coûteuse en temps de calcul qu'une heuristique moins informée.

# Le jeu du Taquin

## Contexte

Prenons l'exemple simple du jeu de taquin de côté 8 ([n-puzzle](https://en.wikipedia.org/wiki/15_puzzle)) dont on rappelle ci-dessous le voisinage.

![eightpuzzle](media/eightpuzzle.png)

## Heuristiques

Voici quelques exemples d'heuristiques ordonnées par degré d'information :

 + nombre de chiffres mal placés
 + somme des distances (de manhattan) des chiffres à leurs positions d'origine
 + prise en compte des échanges entre cases voisines
 + ...

## Implémentation

Nous commençons par définir le type d'un nœud et d'une fonction heuristique.

```c++
#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <map>
#include <utility>
#include <list>
#include <functional>
#include <cmath>
#include <algorithm>
#include <utility>
#include <iomanip>
#include <limits>

using namespace std;

typedef vector<int> State;

//  .---.
//  |2|0|
//  .---. -> State b = {2,0,1,3}
//  |1|3|
//  .---.

typedef function<int( const State& pos )> Heuristic;
```

Nous introduisons une fonction accessoire pour calculer la taille du côté d'une grille.

```c++
int 
side( const State& b )
{
  double y = sqrt( b.size() );
  int x = y;
  return x;
}
```

Puis nous définissons trois heuristiques.

```c++
int 
breadth( const State& b )
{
  return 0;
}

int
nbmis( const State& b )
{
  int d = 0;
  for( int i = 0 ; i < b.size() ; i++ )
  {
    if( b[i] != 0 ) // '0' is not a tile
    {
      if( b[i] != i ) d++;
    }
  }
  return d;
}

int 
manh( const State& b )
{
  int d = 0;
  int s = side(b);
  for( int i = 0 ; i < b.size() ; i++ )
  {
    if( b[i] != 0 )  // not a tile, '0' doesn't count
    {
      d += abs( i / s - b[i] / s ) +
           abs( i % s - b[i] % s );
    }
  }
  return d;
}
```

La fonction `final_state` indique si la configuration finale est atteinte.

```c++
bool
final_state( const State& b )
{
  return (nbmis(b) == 0); // we use nbmis for it is quick to compute
}
```

Nous utilisons une fonction pour retourner la plus petite distance actuellement connue pour rejoindre un nœud donné. Dans le cas d'un nœud pour lequel aucune estimation de distance n'a pour l'instant été proposée, cette fonction retourne l'infini.

```c++
int
dist( const map<State,int>& d, const State& state )
{
  map< State , int >::const_iterator it = d.find(state);
  if( d.end() == it )
  {
    return numeric_limits<int>::max();
  }
  return it->second;
}
```

Nous adaptons l'algorithme $A^*$ (i.e. Dijkstra...) pour le jeu du taquin.

Nous introduisons une fonction pour calculer le voisinage d'une position.

```c++
void
update_neighborhood( const State& current_state, vector<State>& neighbors, int s)
{
  // parameter s is the length of the side of the grid
  neighbors.clear();

  int pos0 = find( current_state.begin(), current_state.end(), 0 ) -
             current_state.begin();
  
  if( (pos0 + 1) < current_state.size() &&
     ((pos0 + 1) % s) != 0 )
  {
    State neighbor = current_state;
    swap( neighbor[pos0] , neighbor[pos0 + 1] );
    neighbors.push_back(neighbor);
  }
  
  if( (pos0 - 1) >= 0 &&
     ((pos0 - 1) % s) != (s-1) )
  {
    State neighbor = current_state;
    swap( neighbor[pos0] , neighbor[pos0 - 1] );
    neighbors.push_back(neighbor);
  }
  
  if( (pos0 + s) < current_state.size() )
  {
    State neighbor = current_state;
    swap( neighbor[pos0] , neighbor[pos0 + s] );
    neighbors.push_back(neighbor);
  }
  
  if( (pos0 - s) >= 0 )
  {
    State neighbor = current_state;
    swap( neighbor[pos0] , neighbor[pos0 - s] );
    neighbors.push_back(neighbor);
  }
}
```

Le corps de l'algorithme devrait être sans surprises.

```c++
void
astar( const State& initial_state, 
       Heuristic     h,
       list<State>&  best_path,
       int&          nbiter )
{
  int grid_length = side(initial_state);

  map< State , State > parent;            // the structure parent is used to trace back
  parent[initial_state] = initial_state;  // the path taken to reach the target
  
  map< State , int > d;                   // d[p] is an estimate of the distance
  d[initial_state] = h(initial_state);    // between position p and the target.
  
  set< State > black;
  
  priority_queue< pair< int , State >,
                  vector< pair< int, State > >,
                  greater< pair< int, State > > > grey;
  grey.push( { d[initial_state], initial_state } ); // initially the source is grey
  
  vector<State> neighbors;
  
  nbiter = 0; // count how many positions had to be explored before finding the
                 // shortest path from the source to the target

  while( !grey.empty() )
  {
    State current_state = grey.top().second;

    nbiter++;

    if( final_state(current_state) )
    {
      best_path.clear();
      while( current_state != initial_state )
      {
        best_path.push_front(current_state);
        current_state = parent[current_state];
      }
      best_path.push_front(initial_state);
      return;
    }

    black.insert(current_state);
    grey.pop();

    update_neighborhood(current_state, neighbors, grid_length);

    for( State neighbor : neighbors )
    {
      // when neighbor n of u is black, there's nothing to do
      if( black.end() != black.find(neighbor) ) continue;
      // when it is white or grey, we may update the shortest known distance to it
      int new_cost = d[current_state] + 1 - h(current_state) + h(neighbor);
      if( dist(d,neighbor) > new_cost )
      {
        d[neighbor] = new_cost;
        parent[neighbor] = current_state;
        grey.push( { d[neighbor] , neighbor } );
      }
    }

  }
}
```

Nous introduisons une fonction pour afficher une grille.

```c++
void
print( const State& state )
{
  int s = side(state);
  for( int i = 0 ; i < state.size() ; i++ )
  {
    if( i % s == 0 ) cout << endl;
    cout << setw(2) << setfill('0') << state[i] << " , ";
  }
  cout << endl;
}
```

Nous testons le programme sur une petite grille de côté 8.

```c++
int
main()
{
  State b = {4,8,3,2,0,7,6,5,1};
  list<State> best_path;
  int nbiter = 0;
  
  astar(b, breadth, best_path, nbiter);
  cout << "Heuristic breadth:" << endl;
  cout << "nb moves: " << best_path.size()-1 << endl;
  cout << "nb nodes explored: " << nbiter << endl;
  
  /*
  for( list<State>::iterator it = best_path.begin() ;
       it != best_path.end() ; it++ )
  {
    print( (*it) );
  }
  */

  best_path.clear();
  astar(b, nbmis, best_path, nbiter);
  cout << "Heuristic nbmis:" << endl;
  cout << "nb moves: " << best_path.size()-1 << endl;
  cout << "nb nodes explored: " << nbiter << endl;
  
  /*
  for( list<State>::iterator it = best_path.begin() ;
       it != best_path.end() ; it++ )
  {
    print( (*it) );
  }
  */

  best_path.clear();
  astar(b, manh, best_path, nbiter);
  cout << "Heuristic manh:" << endl;
  cout << "nb moves: " << best_path.size()-1 << endl;
  cout << "nb nodes explored: " << nbiter << endl;

  /*
  for( list<State>::iterator it = best_path.begin() ;
       it != best_path.end() ; it++ )
  {
    print( (*it) );
  }
  */

  return 0;
}
```

Nous trouvons une solution optimale en 20 coups. En fonction de l'heuristique utilisée, le nombre de nœuds explorés change significativement.

| heuristique | nb nœuds explorés |
| ----------- | ----------------- |
| breadth     | 44696             |
| nbmis       | 2877              |
| manh        | 189               |

## Passage à l'échelle

Même avec l'heuristique `manh`, si nous testons notre programme sur des instances de taille 15 (i.e., des grilles de côté 4), nous observons qu'il ne trouve parfois pas de solution en un temps raisonnable.

```
// C0 small problem easily solved by A*
State b = {4,8,3,2,0,7,6,5,1};

// C1 easy configuration that can also be solved by A*
State b = {7,11,8,3,14,0,6,15,1,4,13,9,5,12,2,10};

// C2 more difficult configuration that may not be solved by A*
State b = {14,10,9,4,13,6,5,8,2,12,7,0,1,3,11,15};

```

L'algorithme A* en tant que variante de la recherche en largeur est gourmand en mémoire (toutes les configurations de coût inférieur à la solution optimale sont mémorisées).