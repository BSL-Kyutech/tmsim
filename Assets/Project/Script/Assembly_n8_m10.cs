using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;

using UnityEngine.SceneManagement;

/// <summary>
/// Class <c>Assembly</c> automatically instantiates a tensegrity manipulator with designated parameters.
/// For use, place an Empty object to where the base of the tensegrity manipulator should be, and attach this script.
/// </summary>
///
public class Assembly_2 : Assembly
{
    // Array to keep other instances of prefabs.
    private GameObject[] baseblocks;
    private GameObject[] loops;
    private GameObject[] stripes;
    private Transform[] structsEnd1;
    private Transform[] structsEnd2;
    
    void Start()
    {
        Time.timeScale=1.0f;

        //radiusLoopが内部に存在していた場合取得
        
        if (!initializer){
            if(PlayerPrefs.HasKey("RadiusLoop")){
            //radiusLoop=PlayerPrefs.GetFloat("RadiusLoop");
            }

            if(PlayerPrefs.HasKey("SpringForce")){
                springForce =PlayerPrefs.GetFloat("SpringForce");
            }



            if(PlayerPrefs.HasKey("edge")){
        
                //edgeparameter_layers=PlayerPrefs.GetFloat("edge");
            }
            if(PlayerPrefs.HasKey("mode")){
                //mode=PlayerPrefs.GetInt("count");
            }
        }
        
        



       

        this.edgeLoop = new float[numLayer];
        this.StrutScale=new float[numLayer];

        int bases = numLayer/5;

        //高さ計算
        //var result=simulateTensegritylength(diffphi,diffpsi,diffRadius);
        //StrutScale=result.scale;
        //predict=result.high;
        //var sumHigh = new float[numLayer];
        //sumHigh=result.highs; 
        //Debug.Log(result);

    
        
        // Allocate variables                           //(オブジェクトの入れものとなる配列生成)
        this.baseblocks = new GameObject[numPrism];          //土台の原点の生成(地面の四角のやつ)
        this.struts = new GameObject[numPrism*numLayer];     //柱の生成
        this.structsEnd1 = new Transform[numPrism*numLayer];
        this.structsEnd2 = new Transform[numPrism*numLayer];
        this.loops = new GameObject[(numLayer+1)];           //柱の点の丸いやつ
        this.stripes = new GameObject[numPrism*2];           //筋肉(紐)の数(白い紐)

        //diffphi=(float)((edgeparameter_layers-1.0)/numLayer);
        

        float first_edge=0.04676555f;
        float last_edge=0.03369787f;
        float onlyedge=0.06268743f;

        if(initializer){

            //手先位置の直径
            float handPosDiameter=0.2f;

            //手先一の高さ
            float maxHigh=1.5f;
        
            //傾き
            float gradient=0.0f;
            gradient=(0.44f-handPosDiameter)/(0-maxHigh);

            float diameters=0.0f;
            /*
            for (int i=1;i<=numLayer;i++){
                //edgeLoop[i-1]= (2.0f*(radiusBase - diffRadius*((float)(i)))*(float)Math.Sin(Math.PI/(2*numPrism)))*0.65f;
                
                
                diameters=gradient*(1.5f/(float)numLayer)*(float)i+0.44f;

                //StrutScale[i-1]=0.80f-(0.80f-0.1f)/numLayer * (i-1);
                StrutScale[i-1]=0.9f;
                //StrutScale[i-1]=0.85f-(0.8W5f-0.7f)/numLayer * (i-1);
                if(i==numLayer){
                    edgeLoop[i-1]=1.2f*diameters*(float)Math.Sin(Math.PI/(2*numPrism));
                }
                
                else{
                    //edgeLoop[i-1]=(first_edge-(first_edge-last_edge)/(numLayer-2) * (i-1))*0.5f;
                    edgeLoop[i-1]=diameters*(float)Math.Sin(Math.PI/(2*numPrism))*0.8f;
                    edgeLoop[i-1]=0.06f;
                }
            }
            */
            //StrutScale=new float[13]{0.5066695f,0.4845017f,0.4922935f,0.4921669f,0.5023773f,0.5407119f,0.5237533f,0.5343785f,0.5101283f,0.5261283f,0.5105029f,0.5381283f,0.5263783f};

            
            //edgeLoop=new float[13]{0.05042118f,0.05176077f,0.0528837f,0.03907325f,0.03636284f,0.03176075f,0.028792f,0.02953992f,0.02667118f,0.02521077f,0.02398368f,0.02277328f,0.03694427f};



            //パラメータの引継ぎ
            for(int i=0;i<numLayer;i++){
                PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
                PlayerPrefs.Save();
            }
            
           

            //edgeLoop=new float[10]{0.065649f, 0.05787f, 0.05438851111111f, 0.05100837766666666f, 0.04738228702222222f, 0.043882287022222224f, 0.039228702222222f, 0.03882287022222223f, 0.03838228702222223f, 0.054f};
            //edgeLoop=new float[10]{0.065649f, 0.06289442f, 0.0731718f, 0.04244859f, 0.04472593f, 0.0510033f, 0.05128067f, 0.04755802f, 0.04683538f, 0.03011275f};
            //edgeLoop=new float[10]{0.05314891f, 0.04836987f, 0.04488838f, 0.04350825f, 0.03888217f, 0.03488217f, 0.02922862f, 0.03382276f, 0.03138219f, 0.04899989f};
            initializer=false;
            //PlayerPrefs.SetInt("compareFlag",0);//compareflagの初期化は後でコメントアウト外さないといけない
            //PlayerPrefs.Save();
            PlayerPrefs.SetInt("EdgeFlag",0);
            PlayerPrefs.Save();
            PlayerPrefs.SetInt("countUnchange",1);
            PlayerPrefs.Save();
            PlayerPrefs.SetFloat("diffloss",10000.0f);
            PlayerPrefs.SetInt("changed",0);
            PlayerPrefs.Save();
            PlayerPrefs.SetInt("photonumber",0);
            PlayerPrefs.Save();
        }
    }
}
