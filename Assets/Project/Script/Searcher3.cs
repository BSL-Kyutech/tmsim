using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

using UnityEngine.SceneManagement;

public class Searcher3 : MonoBehaviour
{

    private float ReturnCorrectRadius(float y){
        //4-5の際の半径
        float [] trueRadius={0.3232192f,0.2661974f,0.2253503f,0.1690026f,0.1804611f};
        //4-5の際の層ごとの高さ
        float [] trueHigh={0.3345878f,0.5892965f,0.8675417f,1.15422f,1.48762f};
        float radius=0.0f;
        radius=trueRadius[0];
        return radius;
    }

    //ばね定数の変更処理
    private float ConculateSpringsLoss(){
        float loss=0.0f;
        return loss;
    }

    //ズレの検出とその損失
    private float ConcurateDeviationLoss(){
        float loss=0.0f;
        return loss;
    }

    //高さの損失計算
    private float ConcurateHighLoss(){
        float loss=0.0f;
        return loss;
    }

    //半径の損失計算
    private float ConcurateRadiusLoss(){
        float loss=0.0f;
        return loss;
    }

    // Start is called before the first frame update
    void Start()
    {           
        StartCoroutine(DelayMethod());
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    IEnumerator DelayMethod(){

        //Asembly.csの取得
        GameObject tm_g =GameObject.Find("tm_g");
        Assembly assembly;
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);

        //レイヤー数の取得
        int numLayer=assembly.numLayer;
        int numPrism=assembly.numPrism;
        float radiusBase=assembly.radiusBase;

        //手先位置の取得変数
        float tm_g_x=0.0f;
        float tm_g_y=0.0f;
        float tm_g_z=0.0f;
        float[] tmpos =new float[3];

        //高さの取得
        float[] tm_g_layers_y;
        tm_g_layers_y = new float[numLayer];


        GameObject[] struts;
        GameObject[] ends;
        struts = new GameObject[numLayer];
        ends = new GameObject[numLayer];

        yield return new WaitForSeconds(2.55f);

        //レイヤー層ごとの高さ
        var strut=tm_g.transform.Find("Strut"+(0).ToString()).gameObject;;
        var end=strut.transform.Find("end");

        //x,y,z座標の取得用配列
        float[] LayerXpositions;
        float[] LayerYpositions;
        float[] LayerZpositions;

        //配列長さはレイヤーが上下の柱より構成されるため2倍
        LayerXpositions= new float [numPrism*2];
        LayerYpositions= new float [numPrism*2];
        LayerZpositions= new float [numPrism*2]; 
        //試行回数
        int operateFor=0;
        //一番長い距離
        float maxDistance=0;

        //直径の記録
        float[] diameters;
        diameters=new float [numLayer];

        //ずれの検出の計算用変数
        float errors=0.0f;

        //一番誤差のあるレイヤーナンバー
        int maxErrorLayerNo=-1;
        //レイヤー内で一番誤差のある長さ
        float maxError=0.0f;
        float[] maxErrors;
        maxErrors=new float [numLayer];

        //層ごとの高さ，半径，ズレの検出
        for(int i=0;i<numLayer;i++){

            //下から生えている柱からend1のx,y,z座標取得する
            for(int j=0;j<numPrism;j++){
                //Debug.Log(j+i*numPrism);
                strut=tm_g.transform.Find("Strut"+(j+i*numPrism).ToString()).gameObject;
                
                if(i==0){
                    //baseStrutなので，end1ではなくendのみ
                    end=strut.transform.Find("end");
                }
                else{
                    //Strutのためend1
                    end=strut.transform.Find("end1");
                }
                //平均計算のためにendたちのyの高さを足し合わせて行く
                tm_g_y+=end.transform.position.y;
                //比較するため配列に代入
                LayerXpositions[j]=end.transform.position.x;
                LayerYpositions[j]=end.transform.position.y;
                LayerZpositions[j]=end.transform.position.z;
                //Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
            }
            
            
             
            //上から刺さっている柱からend2x,y,z座標取得する(一番上は除外)
            if(i==(numLayer-1)){
                operateFor=numPrism;
                //一番上の層のため，上から刺さっている柱が存在しないため，柱の数のみで除算してその高さの平均を求める
                tm_g_y/=numPrism;
            }
            else{
                for(int j=0;j<numPrism;j++){
                    Debug.Log(j+(i+1)*numPrism);
                    strut=tm_g.transform.Find("Strut"+(j+(i+1)*numPrism).ToString()).gameObject;
                    end=strut.transform.Find("end2");

                    //平均計算のためにendたちのyの高さを足し合わせて行く
                    tm_g_y+=end.transform.position.y;

                    LayerXpositions[j+numPrism]=end.transform.position.x;
                    LayerYpositions[j+numPrism]=end.transform.position.y;
                    LayerZpositions[j+numPrism]=end.transform.position.z;
                    //Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
                    //Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
                }
                operateFor=numPrism*2;

                //上にも層があるため，上から刺さっている柱により2倍に
                tm_g_y=tm_g_y/(2*numPrism);
            }
            

            Debug.Log(operateFor.ToString()+" zure");
            
            //半径の計算
            for(int j=0;j<operateFor;j++){
                
                //XとZのそれぞれの距離
                tm_g_x=(float)Math.Pow((double)(LayerXpositions[0]-LayerXpositions[j]),2);
                tm_g_z=(float)Math.Pow((double)(LayerZpositions[0]-LayerZpositions[j]),2);

                if(maxDistance<=(float)(Math.Sqrt(tm_g_x+tm_g_z))){
                    maxDistance=(float)Math.Sqrt(tm_g_x+tm_g_z);
                }
            }
            
            Debug.Log(i.ToString());
            //配列にそれぞれの直径を格納
            diameters[i]=maxDistance;
            maxDistance=0;


            //ズレの検出の計算
            for(int j=0;j<operateFor;j++){
                for(int k=0;k<operateFor;k++){
                    //y座標のそれぞれの距離で一番離れているところを探す
                    errors=0.0f;
                    errors=(float)Math.Sqrt(Math.Pow((double)(LayerYpositions[k]-LayerYpositions[j]),2));
                    
                    if(maxError<=errors){
                        //Debug.Log("maxError"+errors.ToString());
                        maxError=errors;
                    }
                }
                
                
            }

            //配列にそれぞれの層の高さを格納
            tm_g_layers_y[i]=tm_g_y;
            tm_g_y=0;

            Debug.Log(i.ToString());
            //配列にそれぞれのずれの誤差を格納
            maxErrors[i]=maxError;
            maxError=0.0f;

            Array.Fill(LayerXpositions,0.0f);
            Array.Fill(LayerYpositions,0.0f);
            Array.Fill(LayerZpositions,0.0f);

        }

        

        
        for(int i=0;i<numLayer;i++){
            Debug.Log("Layer" +i.ToString()+ "'s high  : "+tm_g_layers_y[i].ToString());
        }

        for(int i=0;i<numLayer;i++){
            Debug.Log("Layer" +i.ToString()+ "'s radius  : "+diameters[i].ToString());
        }
        for(int i=0;i<numLayer;i++){
            Debug.Log("Layer" +i.ToString()+ "'s zure  : "+maxErrors[i].ToString());
        }
    }

    

    
}
 