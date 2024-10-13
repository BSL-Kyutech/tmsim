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
        float [] trueRadius={0.30379696284823254f,0.25485215839600045f,0.21799613961084838f,0.18262210691976397f,0.16326623536043514f};
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
        //層ごとの半径計算
        for(int i=0;i<numLayer;i++){
            //柱の数列考えればエラーは解消されると思われる

            //下から生えている柱からend1の取得する
            for(int j=0;j<numPrism;j++){
                Debug.Log(j+i*numPrism);
                strut=tm_g.transform.Find("Strut"+(j+i*numPrism).ToString()).gameObject;
                
                if(i==0){
                    //baseStrutなので，end1ではなくendのみ
                    end=strut.transform.Find("end");
                }
                else{
                    //Strutのためend1
                    end=strut.transform.Find("end1");
                }
                //合計するための変数に合算
                tm_g_y+=end.transform.position.y;
                Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
            }
            
            //一番上の層のため，上から刺さっている柱が存在しないため，柱の数のみで除算してその高さの平均を求める
            if(i==(numLayer-1)){
                tm_g_y/=numPrism;
            }
            //上から刺さっている柱からend2を取得する
            else{
                for(int j=0;j<numPrism;j++){
                    Debug.Log(j+(i+1)*numPrism);
                    strut=tm_g.transform.Find("Strut"+(j+(i+1)*numPrism).ToString()).gameObject;
                    end=strut.transform.Find("end2");
                

                    tm_g_y+=end.transform.position.y;
                    Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
                }
                //平均の高さを計算
                tm_g_y=tm_g_y/(2*numPrism);
            }
            
            

            //配列にそれぞれを格納
            tm_g_layers_y[i]=tm_g_y;
            tm_g_y=0;

        }
        for(int i=0;i<numLayer;i++){
            Debug.Log("Layer" +i.ToString()+ "'s high  : "+tm_g_layers_y[i].ToString());
        }


        
        



    }

    
}
 