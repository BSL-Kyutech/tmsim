using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.IO;

using UnityEngine.SceneManagement;

public class broken_searcher : MonoBehaviour
{

    private int counts=0;
    private GameObject tm_g;
    private Assembly assembly;
    private Device device;
    private GameObject ctrl;
    private Demo demo;



    

    //csv 出力
    private void OutputCsv(string path,string savedata ){

        //File.AppendAllText("C:/Users/Yamauchi Gaito/Desktop/workspace/tmsim/data/data.csv",savedata);
        File.AppendAllText(path,savedata);
    }

    //1-40を壊す際にランダムに壊す(numLayer*numPrism)から壊れるパターンを取る
    //適当に10個くらいとる？
    //1,2,3,4,5,6..と増やす
    //40以上もやる．
    //定量化するために，いろいろをやる．
    //仮説としては，壊れてる率が影響する
    //40は中間発表まで

    //座標取得
    private (float[] LayerXpositions,float[] LayerYpositions,float[] LayerZpositions,int arrayLength) getPosition(int layerNum,string manipulatorName){
        
        //Asembly.csの取得
        GameObject tm_g =GameObject.Find(manipulatorName);
        Assembly assembly;
        assembly=tm_g.GetComponent<Assembly>();

        var strut=tm_g.transform.Find("Strut"+(0).ToString()).gameObject;
        var end=strut.transform.Find("end");

        int numLayer=assembly.numLayer;
        int numPrism=assembly.numPrism;

        //x,y,z座標の取得用配列
        float[] LayerXpositions;
        float[] LayerYpositions;
        float[] LayerZpositions;
        int arrayLength=0;

        //配列長さはレイヤーが上下の柱より構成されるため2倍
        LayerXpositions= new float [numPrism*2];
        LayerYpositions= new float [numPrism*2];
        LayerZpositions= new float [numPrism*2]; 

        //下から生えている柱からend1のx,y,z座標取得する
        for(int j=0;j<numPrism;j++){
            //Debug.Log(j+i*numPrism);
            strut=tm_g.transform.Find("Strut"+(j+layerNum*numPrism).ToString()).gameObject;
                
            if(layerNum==0){
                //baseStrutなので，end1ではなくendのみ
                end=strut.transform.Find("end");
            }
            else{
                //Strutのためend1
                end=strut.transform.Find("end1");
            }
            //比較するため配列に代入
            LayerXpositions[j]=end.transform.position.x;
            LayerYpositions[j]=end.transform.position.y;
            LayerZpositions[j]=end.transform.position.z;               
            //Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
        }

        //上から刺さっている柱からend2x,y,z座標取得する(一番上は除外)
        if(layerNum==(numLayer-1)){
            arrayLength=numPrism;
        }
        else{
            for(int j=0;j<numPrism;j++){
                //Debug.Log(j+(layerNum+1)*numPrism);
                strut=tm_g.transform.Find("Strut"+(j+(layerNum+1)*numPrism).ToString()).gameObject;
                end=strut.transform.Find("end2");

                

                LayerXpositions[j+numPrism]=end.transform.position.x;
                LayerYpositions[j+numPrism]=end.transform.position.y;
                LayerZpositions[j+numPrism]=end.transform.position.z;
                //Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
                //Debug.Log("i"+i.ToString()+",j"+j.ToString()+",Strut"+(j*i).ToString()+",tm_g_y"+end.transform.position.y.ToString());
                }
            arrayLength=numPrism*2;
            string output=""; 
            for(int i=0;i<LayerXpositions.GetLength(0);i++){
                output=output+"x"+i.ToString()+","+LayerXpositions[i]+",y"+i.ToString()+","+LayerYpositions[i]+",z"+i.ToString()+","+LayerZpositions[i]+"\n";
            }

            
            string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/mountingSearcPositions.csv";
            OutputCsv(path,output);
        }

            
        
        return (LayerXpositions,LayerYpositions,LayerZpositions,arrayLength);
    }

    

    //オブジェクトの先端部の取得
    void getHandPositions(int processNum){
        
        
        //Debug.Log(assembly.StrutScale);
        int numLayer=assembly.numLayer;
        int numPrism=assembly.numPrism;

        //0から計測のため，numLayer-1となる．
        var positionRetrun = getPosition(numLayer-1,"tm_g_1");

        //x,y,z座標の取得用配列
        float[] LayerXpositions;
        float[] LayerYpositions;
        float[] LayerZpositions;

        //配列長さはレイヤーが上下の柱より構成されるため2倍
        LayerXpositions= new float [numPrism*2];
        LayerYpositions= new float [numPrism*2];
        LayerZpositions= new float [numPrism*2]; 

        //座標格納
        LayerXpositions=positionRetrun.LayerXpositions;
        LayerYpositions=positionRetrun.LayerYpositions;
        LayerZpositions=positionRetrun.LayerZpositions;

        //平均値の格納
        float xAverage=0.0f;
        float yAverage=0.0f;
        float zAverage=0.0f;

        //平均値の算出(手先位置の座標を知るため)
        for(int i=0;i<numPrism*2;i++){
            xAverage+=LayerXpositions[i];
            yAverage+=LayerYpositions[i];
            zAverage+=LayerZpositions[i];
        }
        xAverage=xAverage/(numPrism*2);
        yAverage=yAverage/(numPrism*2);
        zAverage=zAverage/(numPrism*2);



        //csv出力

        float timeNow=Time.realtimeSinceStartup;
        
        
        
        string path="C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/broken/handPosition_layer_"+numLayer.ToString()+"_prism_"+numPrism.ToString()+"_process_Count"+processNum.ToString()+".csv";
        if(counts==0){
            //データの最初のラベル配置(t,x,y,z)
            string label="time,x,y,z,";

            //壊れているかのラベル
            for(int i=0;i<2*numLayer*numPrism;i++){
                label=label+"cylinder"+i.ToString()+"broken,";   
            }
            //シリンダのインプット
            for(int i=0;i<2*numLayer*numPrism;i++){
                label=label+"cylinder"+i.ToString()+" Spring";
                if(i!=2*numLayer*numPrism-1){
                    //一番最後はいらないため除外
                    label=label+",";
                }
            }
            label=label+"\n";
            
            OutputCsv(path,label);
        }

        //csv出力
        
        string savedata=Time.timeSinceLevelLoad.ToString()+","+xAverage+","+yAverage+","+zAverage+",";
        //シリンダのアウトプットの記録
        for(int i=0;i<2*numLayer*numPrism;i++){
            if(device.brokenCylinder[i]){
                //pyhtonの表記に合わせるため，大文字に
                savedata=savedata+"True,";
            }
            else{
                savedata=savedata+"False,";
            }
        }
        //シリンダのアウトプットの記録
        //inputだと，シータで変化する部分の上書き前のを取るため，シリンダのばね定数を取る
        for(int i=0;i<2*numLayer*numPrism;i++){
            savedata=savedata+device.cylinder[i].ToString()+",";
        }

        savedata=savedata+"\n";
        OutputCsv(path,savedata);
        Debug.Log(savedata);

        
    }

    

    IEnumerator getsHandsPositions(int processCount){
        getHandPositions(processCount);

        //0.05秒の待機
        yield return new WaitForSeconds(0.03f);
        
    } 

    private void zeroInputsRandom(int breaksCount){
        //randomに壊すのを決める
        System.Random breakNum = new System.Random();

        //壊れた数を記憶する配列
        int[] breaksArray;
        breaksArray=new int[breaksCount];
        //壊れたシリンダの数
        int num=0;
        //かぶりの判断変数
        bool AlreadyNum=false;

        //指定された数の乱数を入れる
        for(int i=0;i<breaksCount;){
            num=breakNum.Next(2*assembly.numLayer*assembly.numPrism);
            //かぶりをなくす処理
            for(int j=0;j<=i;j++){
                if(num==breaksArray[j]){
                    AlreadyNum=true;
                }
            }
            if(!AlreadyNum){
                breaksArray[i]=num;
                i++;
            }
            //大きかったら無限ループになるので壊す
            if(breaksCount>2*assembly.numLayer*assembly.numPrism){
                i=breaksCount+1;
            }
            AlreadyNum=false;
        }
        

        //deviceのtmに壊れたシリンダの位置を送る 
        for(int i=0;i<breaksCount;i++){
            device.brokenCylinder[breaksArray[i]]=true;
        }
        //breakingMode始動
        device.breaking_Mode=true;
        //0入力の決定
        device.ConstantBroken=true;
        device.BrokenInput=0;
        //手先位置の記録

        

    }

    // Start is called before the first frame update
    void Start()
    {
        //オブジェクトへのアクセス
        //変数の確認

        //Asembly.csの取得
        tm_g =GameObject.Find("tm_g_1");
        assembly=tm_g.GetComponent<Assembly>();

        //cylinderのON OFFを取るため
        device=tm_g.GetComponent<Device>();
        //壊すシリンダの決定

        //ctrlオブジェクトの取得
        ctrl=GameObject.Find("ctrl");
        demo=ctrl.GetComponent<Demo>();
        zeroInputsRandom(10);
    }

    // Update is called once per frame
    void Update()
    {   
        //試行回数の記録
        int processCount=0;
        if(PlayerPrefs.HasKey("ProcessCount")){
            processCount=PlayerPrefs.GetInt("ProcessCount");
        }
        //delayを入れる
        Debug.Log(Time.timeSinceLevelLoad);
        
        if(processCount<=10){
            if(Time.timeSinceLevelLoad<30.0f){
                if(Time.timeSinceLevelLoad>5.0f){
                    //でも開始
                    demo.isActive=true;
                }
                StartCoroutine(getsHandsPositions(processCount));
                counts++;
                //引継ぎ
                PlayerPrefs.SetInt("ProcessCount",processCount);
            
            }
            else{
                //Sceneのリセット
                processCount++;
                PlayerPrefs.SetInt("ProcessCount",processCount);
                SceneManager.LoadScene("SampleScene");
            }
        }
        

    }
}
