using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

using UnityEngine.SceneManagement;

public class Searcher3 : MonoBehaviour
{
    //csv 出力
    private void OutputCsv(string path,string savedata ){

        //File.AppendAllText("C:/Users/Yamauchi Gaito/Desktop/workspace/tmsim/data/data.csv",savedata);
        File.AppendAllText(path,savedata);
    }

    //直径の正解値を返す
    private float ReturnCorrectRadius(float y){
        //4-5の際の直径
        float [] trueRadius={0.3232192f,0.2661974f,0.2253503f,0.1690026f,0.1804611f};
        //4-5の際の層ごとの高さ
        float [] trueHigh={0.3345878f,0.5892965f,0.8675417f,1.15422f,1.48762f};
        float radius=0.0f;
        float gradient=0.0f;
        float intercept=0.0f;

        //それぞれの中間の直径を一次関数で近似
        if(y<=trueHigh[0]){
            gradient=(0.44f-trueRadius[0])/(trueHigh[0]-0.0f);
            intercept=0.0f;
            radius=gradient*(y-0.0f)+intercept;
        }
        else if(y<=trueHigh[1]){
            gradient=(trueRadius[0]-trueRadius[1])/(trueHigh[1]-trueHigh[0]);
            intercept=trueRadius[0];
            radius=gradient*(y-trueHigh[0])+intercept;
        }
        else if(y<=trueHigh[2]){
            gradient=(trueRadius[1]-trueRadius[2])/(trueHigh[2]-trueHigh[1]);
            intercept=trueRadius[1];
            radius=gradient*(y-trueHigh[1])+intercept;
        }
        else if(y<=trueHigh[3]){
            gradient=(trueRadius[2]-trueRadius[3])/(trueHigh[3]-trueHigh[2]);
            intercept=trueRadius[2];
            radius=gradient*(y-trueHigh[2])+intercept;
        }
        else if(y<=trueHigh[4]){
            gradient=(trueRadius[3]-trueRadius[4])/(trueHigh[4]-trueHigh[3]);
            intercept=trueRadius[3];
            radius=gradient*(y-trueHigh[3])+intercept;
        }
        else{
            radius=trueRadius[4];
        }
        
        
        
        return radius;
        
    }

    //高さの正解値を返す
    private float ReturnCorrectHigha(int i,int numLayer){
        //4-5の際の層ごとの高さ
        float [] trueHigh={0.3345878f,0.5892965f,0.8675417f,1.15422f,1.48762f};
        float high=0.0f;
        float gradient=0.0f;
        float intercept=0.0f;

        //今の層がどこに位置するか調べる
        int classifier=numLayer/5;
        i=i+1;

        //それぞれの中間の高さを一次関数で近似
        if(i<classifier+1){
            gradient=(trueHigh[0]-0.0f)/(classifier);
            intercept=0.0f;
            high=gradient*i+intercept;
        }
        else if(i<(classifier*2)+1){
            gradient=(trueHigh[1]-trueHigh[0])/(classifier);
            intercept=trueHigh[0];
            high=gradient*(i-classifier)+intercept;
        }
        else if(i<(classifier*3)+1){
            gradient=(trueHigh[2]-trueHigh[1])/(classifier);
            intercept=trueHigh[1];
            high=gradient*(i-classifier*2)+intercept;
        }
        else if(i<(classifier*4)+1){
            gradient=(trueHigh[3]-trueHigh[2])/(classifier);
            intercept=trueHigh[2];
            high=gradient*(i-classifier*3)+intercept;
        }
        else if(i<(classifier*5)+1){
            gradient=(trueHigh[4]-trueHigh[3])/(classifier);
            intercept=trueHigh[3];
            high=gradient*(i-classifier*4)+intercept;
        }
        else{
            high=trueHigh[4];
        }
        
        //Debug.Log("high"+(i-1).ToString()+"  "+high.ToString());
        
        return high;
    }

    //座標取得
    private (float[] LayerXpositions,float[] LayerYpositions,float[] LayerZpositions,int arrayLength) getPosition(int layerNum,int numPrism,int numLayer){
        
        //Asembly.csの取得
        GameObject tm_g =GameObject.Find("tm_g");
        Assembly assembly;
        assembly=tm_g.GetComponent<Assembly>();

        var strut=tm_g.transform.Find("Strut"+(0).ToString()).gameObject;
        var end=strut.transform.Find("end");

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
        }

            
        
        return (LayerXpositions,LayerYpositions,LayerZpositions,arrayLength);
    }

    private float getHigh(float[] LayerYpositions,int arrayLength){
        float tm_g_y=0.0f;
        for(int i=0;i<arrayLength;i++){
            tm_g_y+=LayerYpositions[i];
        }
        return tm_g_y/(float)arrayLength;
    }

    //直径の検出
    private float getRadius(int arrayLength,float[] LayerXpositions,float[] LayerZpositions){
        float tm_g_x=0.0f;
        float tm_g_z=0.0f;
        float maxDistance=0.0f;
        
        //直径の計算
        for(int j=0;j<arrayLength;j++){
            
            //XとZのそれぞれの距離
            tm_g_x=(float)Math.Pow((double)(LayerXpositions[0]-LayerXpositions[j]),2);
            tm_g_z=(float)Math.Pow((double)(LayerZpositions[0]-LayerZpositions[j]),2);
            //一番離れている点を直径とする
            if(maxDistance<=(float)(Math.Sqrt(tm_g_x+tm_g_z))){
                maxDistance=(float)Math.Sqrt(tm_g_x+tm_g_z);
            }
        }

        return maxDistance;
    }

    //ズレの検出
    private float getDeviation(float[] LayerYpositions,int arrayLength){
        //end同士のyの差
        float errors=0.0f;
        //その中でも最大値
        float maxError=0.0f;
        //ズレの検出の計算
        for(int j=0;j<arrayLength;j++){
            for(int k=0;k<arrayLength;k++){
                //y座標のそれぞれの距離で一番離れているところを探す
                errors=0.0f;
                errors=(float)Math.Sqrt(Math.Pow((double)(LayerYpositions[k]-LayerYpositions[j]),2));
                    
                if(maxError<=errors){
                    //Debug.Log("maxError"+errors.ToString());
                    maxError=errors;
                }
            }   
                
        }
        return maxError;
    }
    
    //Deviationでさらに確認するやつ
    private float getDeviationDetails(float[] LayerYpositions,int arrayLength){
        //end同士のyの差
        float errors=0.0f;
        //その中でも最大値
        float maxError=0.0f;
        //ズレの検出の計算
        for(int j=0;j<arrayLength-1;j++){
            
            //y座標のそれぞれの距離で一番離れているところを探す
            errors=0.0f;
            errors=(float)Math.Sqrt(Math.Pow((double)(LayerYpositions[j]-LayerYpositions[j+1]),2));
                    
            if(maxError<=errors){
                //Debug.Log("maxError"+errors.ToString());
                maxError=errors;
            }
            
                
        }
        //numLayer-1と0の床の確認
        errors=(float)Math.Sqrt(Math.Pow((double)(LayerYpositions[0]-LayerYpositions[arrayLength-1]),2));
        if(maxError<=errors){
            //Debug.Log("maxError"+errors.ToString());
            maxError=errors;
        }
        return maxError;
    }

    //ズレの検出とその損失　優先順位　1
    //損失と動作するかのフラグを返す
    private (float error, bool flag) ConcurateDeviationLoss(float error){
        //閾値は0.05くらい？
        float threshold=0.05f;
        bool flag=false;
        //計算用のロス
        float loss=0.0f;
        if(error>threshold){
            loss=(float)Math.Pow((double)error,2);
            flag=true;
        }

        
        return (loss,flag);
    }

    //ばね定数の変更処理　優先順位　2
    //倒れているかどうか確認
    private (float error, bool flag)  ConculateSpringsLoss(float[] xArray, float[] zArray){

        //平均を求める(手先位置の真ん中の座標)
        float x=0.0f;
        float z=0.0f;
        for(int i=0;i<xArray.Length;i++){
            x+=xArray[i];
            z+=zArray[i];
        }

        x/=xArray.Length;
        z/=zArray.Length;

        //最小二乗法での計算
        float loss=0.0f;
        loss=((float)(Math.Pow((double)(0-x),2))+(float)(Math.Pow((double)(0-z),2)));
        //Debug.Log("xz loss   "+loss.ToString());

        bool flag=true;
        //閾値は0.2くらい？(4-5の際の値が0.5532593のため)
        float threshold=0.02f;

        if(loss<=threshold){
            flag=false;
        }

        //Debug.Log(flag);
        return (loss,flag);
    }

    //すべてのレイヤーの高さの損失計算
    private (float loss, bool flag) ConcurateAllHighLoss(float[] yArray){
        //loss
        float loss=0.0f;
        float targetLength=0.0f;
        bool flag=false;
        
        for (int i=0;i<yArray.Length;i++){
            //レイヤーが目指すべき高さを計算
            targetLength=ReturnCorrectHigha(i,yArray.Length);
            //lossを計算
            loss+=ConcurateHighLoss(yArray[i],targetLength);
        }

        return (loss,flag);
    }

    //高さの損失計算  　優先順位　3
    private float ConcurateHighLoss(float y,float targetLength){


        float loss=0.0f;
        //目標値
        //float targetLength = 1.522883f;

        bool flag=false;

        loss=(float)Math.Pow((double)(targetLength-y),2);
        if(Math.Abs((double)loss)>=0.05){
            flag=true;
        }
        //Debug.Log("高さのロス"+loss.ToString());

        return loss;
    }

    //直径の損失計算　  優先順位　4
    private float ConcurateRadiusLoss(float radius, float y){
        float loss=0.0f;

        float targetRadius=ReturnCorrectRadius(y);

        loss=(float)Math.Pow((double)(targetRadius-radius),2);

        return loss;
    }

    //直径の損失計算(すべて)
    private (float loss, bool flag) ConcurateAllRadiusLoss(float[] layersRadius/*レイヤーごとの直径*/, float[] layersHighs /*レイヤーごとの高さ*/){
        float loss=0.0f; //ロス
        bool flag=false; //あくまで念のためのフラグ

        //配列内のlossの計算
        for (int i=0; i<layersRadius.Length;i++){
            loss += ConcurateRadiusLoss(layersRadius[i],layersHighs[i]);
        }

        return (loss,flag);
    }
    
    private void parameterChanging(bool springFlag, float loss,int numLayer){
        if(springFlag){
            
            float springForce=0.0f;
            //パラメータの取得
            if(PlayerPrefs.HasKey("SpringForce")){
                springForce =PlayerPrefs.GetFloat("SpringForce");
            }
            //パラメータを増加
            springForce+=80.0f;
            //パラメータの引継ぎ
            PlayerPrefs.SetFloat("SpringForce",springForce);
            PlayerPrefs.Save();

            Debug.Log("springForce ++ " + springForce.ToString());
            return;
        }
        else{//Strutとエッジループの+-組み合わせ(4通りを確認し，一番lossが小さいものを探す)
            //Strutの調整かエッジループの調整か確認する
            int compareFlag = 0; //0 Strut+edgeLoop- ,1 Strut+edgeLoop+ ,2 Strut-edgeLoop-,3 Strut-edgeLoop+ 
            float springForce=0.0f;
            float diffphaseSpringForce=0.0f;

            if(PlayerPrefs.HasKey("compareFlag")){
                compareFlag =PlayerPrefs.GetInt("compareFlag");
            }
            
            if(compareFlag==0){
                if(PlayerPrefs.HasKey("SpringForce")){
                    springForce =PlayerPrefs.GetFloat("SpringForce");
                }
            }
            else{
                if(PlayerPrefs.HasKey("SpringForce")){
                    springForce =PlayerPrefs.GetFloat("SpringForce");
                }
                if(PlayerPrefs.HasKey("diffphaseSpringForce")){
                    springForce =PlayerPrefs.GetFloat("diffphaseSpringForce");
                }
            }
            
            //どのedgeLoopか確認する
            int edgeFlag =0;
            if(PlayerPrefs.HasKey("EdgeFlag")){
                edgeFlag =PlayerPrefs.GetInt("EdgeFlag");
            }

            float[] Forces;
            Forces = new float[4];
            


            //パラメータの取得(edgeLoop)
            float[] edgeLoop;
            edgeLoop=new float[numLayer];
            for(int i=0;i<numLayer;i++){
                if(PlayerPrefs.HasKey("edgeLoop"+(i).ToString())){
                    edgeLoop[i]=PlayerPrefs.GetFloat("edgeLoop"+(i).ToString());
                }
            }

            //パラメータの取得(Strut basestrut)
            float strutScale=0.0f;
            float baseScale=0.0f;
            if(PlayerPrefs.HasKey("StrutScale")){
                strutScale=PlayerPrefs.GetFloat("StrutScale");
            }
            if(PlayerPrefs.HasKey("baseScale")){
                baseScale=PlayerPrefs.GetFloat("baseScale");
            }

            //lossの保存
            PlayerPrefs.SetFloat("Loss"+compareFlag.ToString(),loss);
            PlayerPrefs.Save();

            //出力
            string savedata="basescale,"+baseScale.ToString()+",strutScale,"+strutScale.ToString();
            //Debug.Log(tm_g_y);
            for(int i=0;i<numLayer;i++){
                savedata+=",edgeLoop"+i.ToString() +"," +edgeLoop[i].ToString(); 
            }
            //savedata+=",highLoss,"+highLosses.ToString()+",highLoss,"+radiusLoss[edgeFlag].ToString()+"\n";
            savedata+=",loss,"+loss.ToString()+",springForce,"+springForce+"\n";

            Debug.Log(savedata);
            //Csvの出力
            string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/mountingSearc3_5.csv";
            OutputCsv(path,savedata);


            //パラメータの調整
            float damperEdge=0.0005f;
            float damperStrut=0.005f;

            int bases=numLayer/5;
            switch(compareFlag){
                case -1:
                    compareFlag+=1;
                    break;
                case 0://0 Strut+edgeLoop- 
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-damperEdge;
                    if(edgeFlag <= (bases-1)){
                        baseScale=baseScale+damperStrut;
                    }
                    else{
                        strutScale=strutScale+damperStrut/(float)(numLayer);
                    }
                    
                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    break;

                case 1://1 Strut+edgeLoop+ 
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+2.0f*damperEdge;

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    break;

                case 2://2 Strut-edgeLoop-
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-2.0f * damperEdge;
                    if(edgeFlag <= (bases-1)){
                        baseScale=baseScale-2.0f *damperStrut;
                    }
                    else{
                        strutScale=strutScale-2.0f *damperStrut/(float)(numLayer);
                    }
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    break;

                case 3://3 Strut-edgeLoop+ 
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+2.0f*damperEdge;
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    
                    break;
                case 4://データの比較段階(時系列上3のデータがこの際とられている．)
                    //最後の加算が面倒なのでパラメータを最初に戻す
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]- damperEdge;
                    if(edgeFlag <= (bases-1)){
                        baseScale=baseScale+ damperStrut;
                    }
                    else{
                        strutScale=strutScale+ damperStrut/(float)(numLayer);
                    }
                    compareFlag=-1;
                    break;

            }
            //Debug.Log(compareFlag);

            //compareFlag+1されているため，最後のデータは=0の際に取得される。
            if(compareFlag==0){
                Debug.Log("----------------------- Compare phase -------------------------------");
                //以前のlossのデータを取得
                float[] losses;
                losses = new float[4];

                for(int i=0;i<4;i++){
                    losses[i]=10000000000.0f;
                    //変更前のばね定数の保存
                    Forces[i]=springForce;
                    if(PlayerPrefs.HasKey("Loss"+compareFlag.ToString())){
                        //Debug.Log(PlayerPrefs.GetFloat("Loss"+i.ToString()));
                        losses[i]=PlayerPrefs.GetFloat("Loss"+i.ToString());
                    }  
                    if(PlayerPrefs.HasKey("springForce"+i.ToString())){
                        Forces[i]=PlayerPrefs.GetFloat("springForce"+i.ToString());
                    }
                }


                //lossの比較
                float minloss=10000000.0f;
                int mins=0;
                //高さが良いやつを選抜する
                int count=0;

                
                
                for(int i=0;i<4;i++){
                    if(minloss>=losses[i]){
                        minloss=losses[i];
                        mins=i;
                        //Debug.Log(minloss);
                    }
                }

                
                        
                //minlosssを過去のデータと比較．ロスが一個前のやつより小さければ更新
                /*
                float diffloss=10000.0f;
                if(PlayerPrefs.HasKey("diffloss")){
                    diffloss=PlayerPrefs.GetFloat("diffloss");
                }
                if(minloss>=diffloss){
                    Debug.Log("minloss : "+minloss +"  >  diff loss :"  +diffloss);
                    mins=-1;
                    minloss=diffloss;
                    
                }
                */
                
                

                //エラーをもとにパラメータの適用(case 3の際の値になっているので，そこから適用するとどうなる？)
                switch(mins){
                    case -1:
                        Debug.Log("***********************lossが大きいため無変更で次のコンペに入ります*************************");
                        for(int i=0;i<numLayer;i++){
                            edgeLoop[i]=PlayerPrefs.GetFloat("diffedgeLoop"+(i).ToString());
                        }
                        springForce=PlayerPrefs.GetFloat("diffspringForce");
                        strutScale=PlayerPrefs.GetFloat("diffStrutScale");
                        baseScale=PlayerPrefs.GetFloat("diffbaseScale");
                        edgeFlag+=1;

                    
                        break;

                    case 0://0 Strut+edgeLoop- 
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-damperEdge;
                        if(edgeFlag <= (bases-1)){
                            baseScale=baseScale+damperStrut;
                        }
                        else{
                            strutScale=strutScale+damperStrut/(float)(numLayer);
                        }
                        springForce=Forces[0];
                        edgeFlag+=1;
                        break;

                    case 1://1 Strut+edgeLoop+ 
                        if(edgeFlag <= (bases-1)){
                            baseScale=baseScale+1.0f *damperStrut;
                        }
                        else{
                            strutScale=strutScale+damperStrut/(float)(numLayer);
                        }
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+1.0f*damperEdge;
                        springForce=Forces[1];
                        edgeFlag+=1;
                        break;

                    case 2://2 Strut-edgeLoop-
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-1.0f * damperEdge;
                        if(edgeFlag <= (bases-1)){
                            baseScale=baseScale-damperStrut;
                        }
                        else{
                            strutScale=strutScale-damperStrut/(float)(numLayer);
                        }
                        springForce=Forces[2];
                        edgeFlag+=1;
                        break;

                    case 3://3 Strut-edgeLoop+ 
                        if(edgeFlag <= (bases-1)){
                            baseScale=baseScale+1.0f *damperStrut;
                        }
                        else{
                            strutScale=strutScale+damperStrut/(float)(numLayer);
                        }
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-damperEdge;
                        springForce=Forces[3];
                        edgeFlag+=1;
                        break;
                }

                //lossが大きかったときにもとに戻すためのバックアップたち
                for(int i=0;i<numLayer;i++){
                PlayerPrefs.SetFloat("diffedgeLoop"+(i).ToString(),edgeLoop[i]);
                PlayerPrefs.Save();
                }
                PlayerPrefs.SetFloat("diffloss",minloss);
                PlayerPrefs.Save();
                PlayerPrefs.SetFloat("diffspringForce",springForce);
                PlayerPrefs.Save();
                PlayerPrefs.SetFloat("diffStrutScale",strutScale);
                PlayerPrefs.Save();
                PlayerPrefs.SetFloat("diffbaseScale",baseScale);
                PlayerPrefs.Save();
            }

            if(edgeFlag>=numLayer){
                edgeFlag=0;
            }
            
            

            //パラメータの引継ぎ
            for(int i=0;i<numLayer;i++){
                PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
                PlayerPrefs.Save();
            }
            
            PlayerPrefs.SetFloat("SpringForce",1280.0f);//つぶれ始めるから，毎回リセットかければいいのでは？
            PlayerPrefs.Save();
            PlayerPrefs.SetInt("compareFlag",compareFlag);
            PlayerPrefs.Save();
            PlayerPrefs.SetFloat("StrutScale",strutScale);
            PlayerPrefs.Save();
            PlayerPrefs.SetFloat("baseScale",baseScale);
            PlayerPrefs.Save();
            PlayerPrefs.SetInt("EdgeFlag",edgeFlag);
            PlayerPrefs.Save();
            
            
            return;
        }
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
        int arrayLength=0;
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

        float[] radiusLosses;
        radiusLosses = new float [numLayer];

        //層ごとの高さ，直径，ズレの検出
        for(int i=0;i<numLayer;i++){

            //座標取得
            var positionRetrun = getPosition(i,numPrism,numLayer);
            
            //配列の長さの確認
            arrayLength=positionRetrun.arrayLength;
            //座標格納
            LayerXpositions=positionRetrun.LayerXpositions;
            LayerYpositions=positionRetrun.LayerYpositions;
            LayerZpositions=positionRetrun.LayerZpositions;

        
            //配列にそれぞれの直径を格納
            diameters[i]=getRadius(arrayLength,LayerXpositions,LayerZpositions);
            maxDistance=0;


            //配列にそれぞれの層の高さを格納
            tm_g_layers_y[i]=getHigh(LayerYpositions,arrayLength);
            

            //配列にそれぞれの層のずれ(end同士の高さの差)を格納
            maxErrors[i]=getDeviation(LayerYpositions,arrayLength);


            

            //ReturnCorrectHigha(i,numLayer);

            
             
           
        }
        
        //層ごとの半径のロス
        var radiusData=ConcurateAllRadiusLoss(diameters,tm_g_layers_y);
        //Debug.Log("radius Loss"+i.ToString() +"  "+radiusData.loss.ToString());

        //層ごとの高さのロス
        var highLosses = ConcurateAllHighLoss(tm_g_layers_y);

        //手先位置(一番上の高さのロス)
        float handPositionLoss=(float)(numLayer*numPrism/*重み*/)*ConcurateHighLoss(tm_g_layers_y[numLayer-1],1.48762f);

        //倒れているか確認
        var springlosses = ConculateSpringsLoss(LayerXpositions,LayerZpositions);

        //層ごと高さと直径のlossの合成
        float allLoss=highLosses.loss+radiusData.loss+handPositionLoss;
        //float allLoss=highLosses.loss+radiusData.loss;
        //Debug.Log(allLoss);
        
        //パラメータの変更処理
        parameterChanging(springlosses.flag,allLoss,numLayer);


        
        SceneManager.LoadScene("SampleScene");

    }

    

    
}
 