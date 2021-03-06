//===============================================================================//
// Name			 :  andor_aograph.h
// Author(s)	 :  Kourosh Darvish, Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation   :  University of Genoa, Italy - dept. DIBRIS
// Version		 :  Hierarchical
// Description   :  AND/OR graph
//===============================================================================//


#ifndef AOGRAPH_H
#define AOGRAPH_H

#include <algorithm>
#include <fstream>

#include "andor_aonode.h"
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <boost/algorithm/string.hpp>

using namespace std;

//! class "Path" for each unique path traversing the graph from the head to the leaves
class Path
{        
    public:
        int pIndex;                 //!< index of the path
        int pCost;                  //!< overall cost of all the nodes in the path
        bool pComplete;             //!< complete: the path fully traverses the graph
        vector<AOnode*> pathNodes;  //!< set of the nodes in the path
        vector<int> pathArcs;       //!< set of the hyperarcs in the path
        vector<bool> checkedNodes;  //!< checked: the node has been analysed
        vector<bool> checkedHyperarcs;//!< checked: the node has been analysed

        
        //! constructor
		Path(int cost, int index);
//		Path(const Path& new_path);
//		Path& operator=(const Path& new_path);
        //! copy constructor
        Path(const Path &toBeCopied, int index);
        
        //! display path information
        void printPathInfo();
        
        //! add a node in the path
        void addNode(AOnode* node);
        
        //! update the path information (when a node is solved)
        void updatePath(string nameNode, int cost);

        //! update the path information (when a node is solved)
        void updatePathNode(string nameNode, int cost);

        //! update the path information (when a hyper-arc is solved)
        void updatePathHyperarc(string namehyperarc, int cost);


        //! find the feasible node to suggest
        string suggestNode();
        
        //! destructor
		~Path()
		{
			//DEBUG:cout<<endl <<"Destroying Path object" <<endl;
//			for(int i=0;i<pathNodes.size();i++)
//				delete pathNodes[i];
//			pathNodes.clear();
//			pathArcs.clear();
//			checkedNodes.clear();
//			checkedHyperarcs.clear();
		}		
};

//! class "AOgraph" for the AND-OR graph
class AOgraph
{    
    protected:
        //** GRAPH INITIALIZATION **//
        //! add a node in the graph
        void addNode(string nameNode, int cost);
        void addHyperarc(HyperArc* hyperarc);
        
        //! find a node by name
        AOnode* findByName(string nameNode);
        
        HyperArc* findByNameHyperarc(string nameHyperarc);



        //! update the feasibility status of the nodes in the graph
        void updateFeasibility();

        //! update the feasibility status of the nodes in the graph
        void updateHyperarcFeasibility();


		
        //! compute the cost to add to a path
        int computeAddCost(AOnode &node, int hIndex);
        
        //! generate all possible paths navigating the graph
        void generatePaths();
        
        //! set up a graph
        void setupGraph();
        
        //** GRAPH NAVIGATION **//
        //! find the hyperarc connecting a parent to a child node
        HyperArc* findHyperarc(AOnode &parent, AOnode &child, int pathIndex);
        
        //! compute the overall update cost (intermediate step to update the path cost)
        int computeOverallUpdate(AOnode &node);
        
        //! update all paths (update path costs when a node is solved)
        void updatePaths(AOnode &solved);
        
        //! update all paths (update path costs when a node is solved)
        void updatePaths_NodeSolved(AOnode &solved);
        //! update all paths (update path costs when a Hyper-arc is solved)
        void updatePaths_HyperarcSolved(HyperArc &solved);

        //! Find the graph that the a hyper-arc or node belongs to, from the graph name

        AOgraph* findGraph(string graphName);


        //! find the optimal path (long-sighted strategy)
        int findOptimalPath();
        /*! if the optimal path has no feasible hyperarc or node,
         * we go to check other paths, and find the optimal path there.*/
        int findNextOptimalPath(int previousOptimalPathIndex);
       
    
    public:
        string gName;           		//!< name of the graph
        vector<AOnode> graph;   		//!< set of nodes in the AND-OR graph
        vector<HyperArc*> graphHA;		//!< set of hyper-arcs in the AND-OR graph
        HyperArc* upperLevelHyperarc; 	//!< in case of hierarchical and/or graph, from a graph we can go the upper level hyper-arc.
        AOnode* head;           		//!< pointer to the node = final assembly
        vector<Path> paths;     		//!< set of paths in the AND-OR graph
        vector<int> pIndices;   		//!< indices of the updated paths
        vector<int> pUpdate;    		//!< costs subtracted to the updated paths
        vector<string> Nodes_solved_infeasible; //!< Nodes that become infeasible when an hyper-arc is solved
        
        //! constructor
		AOgraph(string name);
//		AOgraph(const AOgraph& new_graph);
//		AOgraph& operator=(const AOgraph& new_graph);
        
        //! load the graph description from a file
        void loadFromFile(string filePath, string fileName);
        
        //! display graph information
        void printGraphInfo();
        
        //! suggest the node to solve
        string suggestNext(bool strategy);
        
        //! solve a node, finding it by name
        void solveByNameNode(string graphName, string nameNode);
        void solveByNameNodeMe(string graphName, string nameNode);
        void solveByNameHyperarcMe(string graphName,string nameHyperarc);

        //! solve a hyperArc, finding it by name
        void solveByNameHyperarc(string graphName,string nameHyperarc);

        //! get feasible nodes lists and costs
        void getFeasibleNode(vector<andor_msgs::Node> &feasileNodeVector);
        //! get feasible hyperarc lists and costs
        void getFeasibleHyperarc(vector<andor_msgs::Hyperarc> &feasileHyperarcVector, vector<andor_msgs::Node> &feasileNodeVector);
        bool isGraphSolved(void);
         vector<string> getSolvedNodes();
        vector<string> getSolvedHyperArcs();
        //! destructor
		~AOgraph()
		{
			//DEBUG:cout<<endl <<"Destroying AOgraph object" <<endl;
//			for(int i=0;i<graphHA.size();i++)
//				delete graphHA[i];
//			graphHA.clear();
//			delete head;
//			paths.clear();
//			pIndices.clear();
//			pUpdate.clear();
//			Nodes_solved_infeasible.clear();
		}

};

#endif
