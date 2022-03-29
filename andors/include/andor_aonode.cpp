//===============================================================================//
// Name			: andor_aonode.cpp
// Author(s)	: Kourosh Darvish, Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: Generic node element of an AND-OR graph
//===============================================================================//

#include "andor_aonode.h"

//! constructor of class HyperArc
//! @param[in] index    index of the hyperarc
//! @param[in] nodes    set of child nodes connected via the hyperarc
//! @param[in] cost     generic hyperarc cost
HyperArc::HyperArc(string name, int index, vector<AOnode*> childNodes, int cost, string fatherName)
{
//	cout<<"HyperArc::HyperArc()"<<endl;
    hIndex = index;
    hCost = cost;
    hName=name;
    hFeasible=false;
    hSolved=false;
    hfatherName=fatherName;

    children = childNodes;
    lowerGraph=NULL;
    includingGraph=NULL;

    //DEBUG:printArcInfo();
}
void HyperArc::SetGraphs(AOgraph* newLowerGraph, AOgraph* sameLevelGraph){
	lowerGraph=newLowerGraph;
	includingGraph=sameLevelGraph;
};


//HyperArc::HyperArc(const HyperArc& new_ha){
//	cout<<"HyperArc::HyperArc(const HyperArc& )"<<endl;
//	hIndex = new_ha.hIndex;
//
//	hCost = new_ha.hCost;
//	hName=new_ha.hName;
//	hFeasible=new_ha.hFeasible;
//	hSolved=new_ha.hSolved;
//	hfatherName=new_ha.hfatherName;
//
//	children = new_ha.children;
//
////	for(int i=0;i<new_ha.children.size();i++)
////	{
//////		AOnode* child=new AOnode(*(new_ha.children[i]));
////		children.push_back(new AOnode( *(new_ha.children[i]) ) );
////	}
//
////	vector<AOnode*> children;
//}
//HyperArc& HyperArc::operator=(const HyperArc& new_ha){
//	cout<<"HyperArc::operator=(const HyperArc& )"<<endl;
//	hIndex = new_ha.hIndex;
//	children = new_ha.children;
//	hCost = new_ha.hCost;
//	hName=new_ha.hName;
//	hFeasible=new_ha.hFeasible;
//	hSolved=new_ha.hSolved;
//	hfatherName=new_ha.hfatherName;
////	for(int i=0;i<new_ha.children.size();i++)
////	{
//////		AOnode* child=new AOnode(*(new_ha.children[i]));
//////		children.emplace_back(make_shared <AOnode*>( *(new_ha.children[i]) ) );
////		children.push_back(new AOnode( *(new_ha.children[i]) ) );
////	}
//	return *this;
//}


//! display hyperarc information
void HyperArc::printArcInfo()
{
	cout<<"Hyperarc name: " <<hName<<endl;
	cout<<"Info of hyperarc: " <<hIndex <<endl;
    cout<<"Hyperarc cost: " <<hCost <<endl;
    cout<<"Hyperarc is feasible?: " <<hFeasible<<endl;
    cout<<"Hyperarc is solved?: " <<hSolved<<endl;

        cout<<"Child nodes: ";
    for (int i=0; i< (int)children.size(); i++)
        cout<<children[i]->nName <<" ";
    cout<<endl;
}

//! set the hyper-arc as solved
//! @param[in] Nodes_solved_infeasible    	set of nodes that become infeasible when an hyper-arc is solved
//! @return result of the operation (true = done, false = not done)
bool HyperArc::setSolved(vector<string> &Nodes_solved_infeasible)
{
    // issue a warning if the hyper-arc is already solved
    if(hSolved == true)
    {
        cout<<"[WARNING] The hyper-arc is already solved." <<endl;
        return false;
    }
    // a hyper-arc can be solved only if it's feasible
    if (hFeasible == true)
    {
        hSolved = true;
        for (int i=0;i<(int)children.size();i++)
        {
        	children[i]->nFeasible=false;
        	Nodes_solved_infeasible.push_back(children[i]->nName);
        }
    }
    else
    {
        cout<<"[ERROR] The hyper-arc is not feasible. Are you sure it is solved?" <<endl;
        return false;
    }
    return true;
}


//! determine whether the hyper-arc is feasible
void HyperArc::isFeasible()
{

	bool temp_isFeasible = false;

//	// 1. the hyper-arc is feasible if it is already feasible
//
//	if (hFeasible == true)
//		temp_isFeasible = true;


	// 2. the hyper-arc is feasible if all child nodes are solved and feasible
	bool allSolved = true;
	for (int i=0; i<(int)children.size(); i++)
	{
		// if the child node is not solved --> break
		if (children[i]->nSolved == false )
		{
			allSolved = false;
			break;
		}
	}

	bool allFeasible= true;
	for (int i=0; i<(int)children.size(); i++)
	{
		if (children[i]->nFeasible== false)
			{
			allFeasible=false;
			break;
			}
	}


	// if all the child nodes are solved, this hyper-arc is feasible
	if (allSolved == true && allFeasible==true)
	{
		temp_isFeasible = true;
	}

	// set the feasibility property
	hFeasible = temp_isFeasible;
//	cout<<"HyperArc::isFeasible: "<<hName<<" "<< hFeasible<<endl;
}



//! constructor of class AOnode
//! @param[in] name	   name of the node
//! @param[in] cost    generic node cost
AOnode::AOnode(string name, int cost)
{
    nName = name;
	nCost = cost;
    nFeasible = false;
    nSolved = false;
    nElement=NULL;
    
    //DEBUG:printNodeInfo();
}

//AOnode::AOnode(const AOnode& new_node){
//	nName = new_node.nName;
//	nCost = new_node.nCost;
//	nFeasible = new_node.nFeasible;
//	nSolved = new_node.nSolved;
//	nElement=new NodeElement(*(new_node.nElement));
//
////	arcs=new_node.arcs;
//	for(int i=0;i<new_node.arcs.size();i++)
//	{
//		HyperArc haToAdd(new_node.arcs[i]);
//		arcs.push_back(haToAdd);
//	}
//
//	for(int i=0;i<new_node.parents.size();i++)
//	{
//		AOnode* parent=new AOnode(*(new_node.parents[i]));
//		parents.push_back(parent);
//	}
//
//}
//AOnode& AOnode::operator=(const AOnode& new_node){
//	nName = new_node.nName;
//	nCost = new_node.nCost;
//	nFeasible = new_node.nFeasible;
//	nSolved = new_node.nSolved;
//	nElement=new_node.nElement;
//	arcs=new_node.arcs;
//
//	for(int i=0;i<new_node.parents.size();i++)
//	{
//		AOnode* parent=new AOnode(*(new_node.parents[i]));
//		parents.push_back(parent);
//	}
//
//	return *this;
//}

//! associate the application-specific element with the node
//! @param[in] element    pointer to the element to associate
void AOnode::addElement(NodeElement* element)
{
    nElement = element;
}

//! add an hyperarc to child nodes
//! @param[in] hyperarcIndex    hyperarc index
//! @param[in] nodes            set of child nodes connected via the hyperarc
//! @param[in] hyperarcCost     hyperarc cost
HyperArc& AOnode::addArc(string hyperarcName, int hyperarcIndex, vector<AOnode*> nodes, int hyperarcCost, string hyperarcFatherName)
{

//	cout<<201<<endl;

//    cout<<202<<endl;
    // add this node to the vector of parents of each child node
    for (int i=0; i< (int)nodes.size(); i++)
        nodes[i]->parents.push_back(this);
    // create the hyperarc
    HyperArc toAdd(hyperarcName, hyperarcIndex, nodes, hyperarcCost, hyperarcFatherName);
    // add it to the set of hyperarcs
    arcs.push_back(toAdd);
//    arcs.push_back(*(new HyperArc(hyperarcName, hyperarcIndex, nodes, hyperarcCost, hyperarcFatherName)));

    return arcs.back();
}

//! display node information
void AOnode::printNodeInfo()
{
    cout<<endl;
    cout<<"Info of node: " <<nName <<endl;
    cout<<"Node cost: " <<nCost <<endl;
    cout<<"Is feasible? " <<boolalpha <<nFeasible <<endl;
    cout<<"Is solved? " <<boolalpha <<nSolved <<endl;
    cout<<"Parent nodes: ";
    for (int i=0; i< (int)parents.size(); i++)
        cout<<parents[i]->nName <<" ";
    cout<<endl;
    cout<<"Number of hyperarcs: " <<arcs.size() <<endl;
    for (int i=0; i< (int)arcs.size(); i++)
        arcs[i].printArcInfo();
    
    // display info of the associated element
    if (nElement != NULL)
    {
        cout<<"Info of associated element" <<endl;
        nElement->printNodeElementInfo();
    }
    else
    {
    	//cout<<"[REPORT] No application-specific element is associated with this node." <<endl;
    }
}

//! determine whether the node is feasible
void AOnode::isFeasible(vector<string> &Nodes_solved_infeasible)
{
	/*! this variable will be true, if the node is solved previously and now is not feasible*/
	bool node_is_infeasible=false;
	bool temp_isFeasible = false;

	for (int j=0;j<(int)Nodes_solved_infeasible.size();j++){
		if (Nodes_solved_infeasible[j]==nName){
			node_is_infeasible=true;
			break;
		}
	}
	if(node_is_infeasible==false)
	{
		// 1. the node is feasible if it is already feasible
		if (nFeasible == true)
			temp_isFeasible = true;

		// 2. the node is feasible if it is a terminal node (no hyperarcs)
		if (arcs.size() == 0)
			temp_isFeasible = true;

		// 3. the node is feasible if it has >=1 solved hyperarcs
		// iterate on the hyperarcs of the node
		for (int i=0; i<(int)arcs.size(); i++)
		{
			if(arcs[i].hSolved==true)
			{
				temp_isFeasible=true;
				break;
			}
		}
	}
	// set the feasibility property
	nFeasible = temp_isFeasible;
}

//! set the node as solved
//! @return result of the operation (true = done, false = not done)
bool AOnode::setSolved()
{
    // issue a warning if the node is already solved
    if(nSolved == true)
    {
        cout<<"[WARNING] The node is already solved." <<endl;
        return false;
    }
    // a node can be solved only if it's feasible
    if (nFeasible == true)
        nSolved = true;
    else
        cout<<"[ERROR] The node is not feasible. Are you sure it is solved?" <<endl;
    
    return nSolved;
}
