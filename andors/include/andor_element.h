//===============================================================================//
// Name			: element.h
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: Base class for the application-specific parameters inside a node
//===============================================================================//

#ifndef ELEMENT_H
#define ELEMENT_H

//! base class "NodeElement" for the application-specific parameters inside a node
class NodeElement
{
    public:    
        //! constructor
		NodeElement()
        {
			//DEBUG:cout<<endl <<"Destroying NodeElement object" <<endl;
		}
        
        //! display element information
        virtual void printNodeElementInfo() = 0;
        
        //! destructor
		~NodeElement()
		{
			//DEBUG:cout<<endl <<"Destroying NodeElement object" <<endl;
		}
};

#endif