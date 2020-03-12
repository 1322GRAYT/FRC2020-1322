package frc.robot.subsystems;

import frc.robot.subsystems.pRFSLIB;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RFSLIB extends SubsystemBase {


	/*********************************/
	/* Dead-Banding Functions        */
    /*********************************/	
	
	public double ApplyDB_Scld(double Le_x_PwrRqstRaw, double Le_x_DB_Thrsh, double Le_x_DB_Lim) {
		return(pRFSLIB.ApplyDB_Scld(this, Le_x_PwrRqstRaw, Le_x_DB_Thrsh, Le_x_DB_Lim));
	}    


	public double ApplyDB_Zeroed(double Le_x_PwrRqstRaw, double Le_x_DB_Thrsh) {
		return(pRFSLIB.ApplyDB_Zeroed(this, Le_x_PwrRqstRaw, Le_x_DB_Thrsh));
	}    


	public double ApplyDB_Clpd(double Le_x_PwrRqstRaw, double Le_x_DB_Thrsh) {
		return (pRFSLIB.ApplyDB_Clpd(this, Le_x_PwrRqstRaw, Le_x_DB_Thrsh));
	}    



   /*********************************/
   /* Normalization Function        */
   /*********************************/
   	
   /** Method: limRateOnInc - This function will rate limit an increase
	  *  delta command based on a maximum limit threshold in delta units
	  * per loop.
      * @param: La_x_ValDsrd[]   (double: )
      * @param: Le_x_ValNormMax, (double: )
	  * @return: La_x_ValDsrd[]  (double: ) */	
	  public double[] NormArrayVals(double La_x_ValDsrd[], double Le_x_ValNormMax) {
		return(pRFSLIB.NormArrayVals(this, La_x_ValDsrd, Le_x_ValNormMax));
	}	



	/*********************************/
	/* Rate Limiting Functions       */
    /*********************************/	
	
    /** Method: LimRateOnInc - This function will rate limit an increase
	  *  delta command based on a maximum limit threshold in delta units
	  * per loop.
      * @param: ValRaw     ( float: Input Value Raw )
      * @param: ValLimPrev ( float: Input Value Limited from Previous Loop)
      * @param: DeltLimMax ( float: Maximum Limit in Delta Change per Loop )
	  * @return: ValLimNew ( float: Updated Input Value Limited) */	
	public double LimRateOnInc(double ValRaw, double ValLimPrev, float  DeltLimMax) {
        return(pRFSLIB.LimRateOnInc(this, ValRaw, ValLimPrev, DeltLimMax));
    }



	/*********************************/
	/* Table Look-Up Functions       */
    /*********************************/	
	
	/** Method: LkUpAxisPWL_int - This function will return an rescaled
	  * axis index value from a Piece-Wise Linear tunable axis of type Integer.
	  * @param1: Axis Input Value in engineering units (float)
	  * @param2: Axis Array Object Reference (reference to array of ints)
	  * @param3: Table Size (int)
	  * @return: Rescaled Axis Output value in a normalized index value (float) */
	public float LkUpAxisPWL_int(float InpVal, int [] AxisArray, int TblSize) {
	    return(pRFSLIB.AxisPieceWiseLinear_int(this, InpVal, AxisArray, TblSize));
	}

	
	/** Method: LkUpAxisPWL_flt - This function will return an rescaled
	  * axis index value from a Piece-Wise Linear tunable axis of type float.
	  * @param: Axis Input Value in engineering units (float)
	  * @param: Axis Array Object Reference (reference to array of floats)
	  * @param: Table Size (int)
	  * @return: Rescaled Axis Output value in a normalized index value (float) */	
	public float LkUpAxisPWL_flt(float InpVal, float[] AxisArray, int TblSize) {
	    return(pRFSLIB.AxisPieceWiseLinear_flt(this, InpVal, AxisArray, TblSize));
	}

	
	/** Method: LkUpAxisLinear_flt - This function will return an interpolated
	 * linear scalar value between two axis limit values (LwrRef and UprRef).
     * @param: Axis Input Value in engineering units (float)
     * @param: Axis Lower Bound Reference Value in engineering units (float)
     * @param: Axis Upper Bound Reference Value in engineering units (float)
	 * @return: Rescaled Axis Output value in a normalized index value (float) */	
	public float LkUpAxisLinear_flt(float InpVal, float LwrRef, float UprRef) {
	    return(pRFSLIB.AxisLinear_flt(this, InpVal, LwrRef, UprRef));
    }
	
		
	/** Method: LkUpTblXY_flt - This function will return an interpolated
	 * table look-up value based on a rescaled normalized axis index value.
     * @param:  TblArray    (float [] Look-Up Table Array Object Reference (reference to array of floats))
     * @param:  AxisInpIdx  (float:   Normalized Axis Index Input value)
     * @param:  TblSize     (int:     Table Size)
	 * @return: Table Look-Up Output value in engineering units (float) */	
	public float LkUpTblXY_flt(float [] TblArray, float AxisInpIdx, int TblSize)
	  {
	      return(pRFSLIB.XY_Lookup_flt(this, TblArray, AxisInpIdx, TblSize));
	  }


	/** Method: XYZ_Lookup_flt - This function will return an interpolated
	 * table look-up value based on a rescaled normalized axis index value.
     * @param1: Look-Up Table Array Object Reference (reference to 3-D array of floats)
     * @param2: Normalized Y-Axis Index Input value (float)
     * @param3: Normalized X-Axis Index Input value (float)
     * @param4: Table Y-Axis Size (int)
     * @param5: Table X-Axis Size (int)
	 * @return: Table Look-Up Output value in engineering units (float) */	
	public float XYZ_Lookup_flt(float [][] TblArray, float AxisInpIdxY, float AxisInpIdxX, int TblSizeY, int TblSizeX)
	{
	    return(pRFSLIB.XYZ_Lookup_flt(this, TblArray, AxisInpIdxY, AxisInpIdxX, TblSizeY, TblSizeX));
	}
	    
}
