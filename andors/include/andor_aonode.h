//===============================================================================//
// Name			: andor_aonode.h
// Author(s)	: Kourosh Darvish, Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: Generic node element of an AND-OR graph
//===============================================================================//

#ifndef AONODE_H
#define AONODE_H

#include <iostream>
#include <vector>

#include "andor_element.h"
#include <boost/shared_ptr.hpp>

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;

// empty declaration (required by HyperArc)
class AOnode;
class AOgraph;

//! class "HyperArc" for the hyperarc connecting one parent node to a number of child nodes in an AND relationship among themselves
class HyperArc
{
    public:
        int hIndex;                 //!< index of the hyperarc
//        vector<shared_ptr<AOnode*>> children;   //!< set of child nodes connected via the hyperarc
        vector<AOnode*> children;   //!< set of child nodes connected via the hyperarc
        int hCost;                  //!< cost of the hyperarc
        string hName;               //!< name of the hyperarc
        string hfatherName;         //!< name of the hyperarc father node
        bool hSolved;               //!< solved: the operation has been performed
        bool hFeasible;             //!< feasible: >=1 hyperarc has all child nodes solved
        AOgraph* lowerGraph;		//!< the pointer to the lower level graph in the hierarchical and/or graph;
        AOgraph* includingGraph;	//!< the pointer to the graph which includes this hyper-arc
        
        //! constructor
		HyperArc(string name, int index, vector<AOnode*> childNodes, int cost, string fatherName);
//		HyperArc(const HyperArc& new_ha);
//		HyperArc& operator=(const HyperArc& new_ha);
        
        //! display hyperarc information
        void printArcInfo();
        
        void SetGraphs(AOgraph* newLowerGraph, AOgraph* sameLevelGraph);

        //! determine whether the hyper-arc is feasible
        void isFeasible();

        //! set the hyper-arc as solved
        bool setSolved(vector<string> &Nodes_solved_infeasible);

        //! destructor
		~HyperArc()
		{
//			delete [] lowerGraph;
			//DEBUG:cout<<endl <<"Destroying HyperArc object" <<endl;
		}
};

//! class "AOnode" for the generic node element of an AND-OR graph
class AOnode
{
    public:
        NodeElement* nElement;      //!< pointer to the application-specific element associated with the node        
        string nName;               //!< name of the node
        int nCost;                  //!< cost of the node
        bool nSolved;               //!< solved: the operation has been performed
        bool nFeasible;             //!< feasible: >=1 hyperarc has all child nodes solved
        vector<HyperArc> arcs;      //!< hyperarcs connecting the node to child nodes
        vector<AOnode*> parents;    //!< nodes having this node as a child node

        //! constructor
		AOnode(string name, int cost);

//		AOnode(const AOnode& new_node);
//		AOnode& operator=(const AOnode& new_node);

        //! associate the application-specific element with the node
        void addElement(NodeElement* element);
        
        //! add an hyperarc to child nodes
        HyperArc& addArc(string hyperarcName, int hyperarcIndex, vector<AOnode*> nodes, int hyperarcCost, string hyperarcFatherName);
        
        //! display node information
        void printNodeInfo();
        
        //! determine whether the node is feasible
        void isFeasible(vector<string> &Nodes_solved_infeasible);
        
        //! set the node as solved
        bool setSolved();
        
        //! destructor
		~AOnode()
		{
			//DEBUG:cout<<endl <<"Destroying AOnode object" <<endl;
//			cout<<401<<endl;
//			for(int i=0;i<parents.size();i++)
//				delete parents[i];
//			cout<<402<<endl;
//			parents.clear();
//			cout<<403<<endl;
//			arcs.clear();
//			cout<<404<<endl;
//			delete nElement;
//			cout<<405<<endl;
		}
};

#endif
