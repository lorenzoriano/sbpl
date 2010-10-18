/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __ENVIRONMENT_NAVXYTHETAMLEVLAT_H_
#define __ENVIRONMENT_NAVXYTHETAMLEVLAT_H_

//these structures contain footprints for the additional levels 
//each of these structures corresponds to one of the EnvNAVXYTHETALATAction_t structures
typedef struct{
	char starttheta; //should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
	char dX;   //should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
	char dY;   //should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
	char endtheta;  //should be equal to the corresponding EnvNAVXYTHETALATAction_t structure
	vector<sbpl_2Dcell_t>* intersectingcellsV; //one footprint per additional level
} EnvNAVXYTHETAMLEVLATAddInfoAction_t;


/**
	*	 \brief This is x,y,theta lattice planning but with multiple levels in z. In other words, it is for doing
	*	 collision checking in 3D (x,y,z). The z level is split into numofzlevs levels. If numofzlevs = 1, then it defaults
	*    to the original x,y,theta lattice planning defined in EnvironmentNAVXYTHETALAT. Otherwise, it uses numofzlevs footprints
	*    of the robot and corresponding costmaps. It assumes that they correspond to each other and are projections of the robot
	*    and corresponding z regions of the 3D map.
*/
class EnvironmentNAVXYTHETAMLEVLAT : public EnvironmentNAVXYTHETALAT
{

public:

  /**
   * \brief initialization of additional levels. 0 is the original one. All additional ones will start with index 1
  */
	bool InitializeAdditionalLevels(int numofadditionalzlevs, const vector<sbpl_2Dpt_t>* perimeterptsV);	

  /**
   * \brief setting 2D map for the additional level at levind index 
   * (indices are zero-based and are only used to index the additional levels)
   * you can not use this function to set 2D map for the base level
   * transform from linear array mapdata to the 2D matrix used internally: Grid2D[x][y] = mapdata[x+y*width]
  */
	bool Set2DMapforAddLev(const unsigned char* mapdata, int levind);

  /**
	 * \brief update the traversability of a cell<x,y> in level zlev
  */
    bool UpdateCostinAddLev(int x, int y, unsigned char newcost, int zlev);

  /**
   * \brief incremental planning not supported
  */
	virtual void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
	{
		printf("ERROR: GetPredsofChangedEdges function not supported\n");
		exit(1);
	}

	
	/**
	 * \brief incremental planning not supported
  */
	virtual void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
	{
		printf("ERROR: GetSuccsofChangedEdges function not supported\n");
		exit(1);
	}

	/**
	 * returns true if cell is traversable and within map limits - it checks against all levels including the base one
    */
	bool IsValidCell(int X, int Y);

	/**
	 * returns true if cell is traversable and within map limits for a particular level
    */
	bool IsValidCell(int X, int Y, int levind);

	
	/**
	 * returns true if cell is untraversable at any level
    */
	bool IsObstacle(int x, int y);

	/**
	 * returns true if cell is untraversable at level levelnum. 
    */
	bool IsObstacle(int x, int y, int levind);

   /**
   * \brief returns false if robot intersects obstacles or lies outside of the map. 
   *  Note this is pretty expensive operation since it computes the footprint
   * of the robot based on its x,y,theta
  */
	bool IsValidConfiguration(int X, int Y, int Theta);

  /** 
   * \brief returns the maximum over all levels of the cost corresponding to the cell <x,y>
  */
	unsigned char GetMapCost(int X, int Y);

  /** 
   * \brief returns the cost corresponding to the cell <x,y> at level levind
  */
	unsigned char GetMapCost(int X, int Y, int levind);


   EnvironmentNAVXYTHETAMLEVLAT();
   ~EnvironmentNAVXYTHETAMLEVLAT();

protected:
	/**
	* \brief number of additional levels. If it is 0, then there is only one level - base level
	*/
	int numofadditionalzlevs; 


	/**
	* \brief footprints for the additional levels
	*/
	vector<sbpl_2Dpt_t>* AddLevelFootprintPolygonV;

	/**
	* \brief array of additional info in actions, 
	* AdditionalInfoinActionsV[i][j] - jth action for sourcetheta = i 
	* basically, each Additional info structure will contain numofadditionalzlevs additional intersecting 
	* cells vector<sbpl_2Dcell_t> intersectingcellsV
	*/
	EnvNAVXYTHETAMLEVLATAddInfoAction_t** AdditionalInfoinActionsV; 


	/**
	* \brief 2D maps for additional levels. 
	* AddLevelGrid2D[lind][x][y] refers to <x,y> cell on the additional level lind
	*/
	unsigned char*** AddLevelGrid2D;


    virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

	virtual int GetActionCostacrossAddLevels(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);


};



#endif
