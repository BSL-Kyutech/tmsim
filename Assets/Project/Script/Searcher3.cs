using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

using UnityEngine.SceneManagement;

public class Searcher3 : MonoBehaviour
{

    private int counts=0;

    //スクショ関数
    void OnScrrenCapture(string path){
        ScreenCapture.CaptureScreenshot(path);
    }


    //csv 出力
    private void OutputCsv(string path,string savedata ){

        //File.AppendAllText("C:/Users/Yamauchi Gaito/Desktop/workspace/tmsim/data/data.csv",savedata);
        File.AppendAllText(path,savedata);
    }

    //直径の正解値を返す
    private float ReturnCorrectDiameter(float y,float DiameterBase){

        //手先位置の直径
        float handPosDiameter=0.2f;

        //手先一の高さ
        float maxHigh=1.5f;
        
        //傾き
        float gradient=0.0f;
        gradient=(DiameterBase-handPosDiameter)/(0-maxHigh);

        float diameters=gradient*y+DiameterBase;

        return diameters;
        
    }

    //高さの正解値を返す
    private float ReturnCorrectHigha(int i,int numLayer){
        

        float maxHigh=1.5f;

        float gradient=maxHigh/numLayer;
        float high=gradient*(i+1);


        
        //Debug.Log("high"+(i-1).ToString()+"  "+high.ToString());
        
        return high;
    }

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

            
            //string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/mountingSearcPositions.csv";
            //OutputCsv(path,output);
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
    private float getDiameter(int arrayLength,float[] LayerXpositions,float[] LayerZpositions){
        float tm_g_x=0.0f;
        float tm_g_z=0.0f;
        float maxDistance=0.0f;

        if(arrayLength%2==1){
            tm_g_x=(float)Math.Pow((double)(LayerXpositions[0]-LayerXpositions[1]),2);
            tm_g_z=(float)Math.Pow((double)(LayerZpositions[0]-LayerZpositions[1]),2);
            maxDistance=(float)Math.Sqrt(tm_g_x+tm_g_z)/((float)Math.Tan(Math.PI/arrayLength));
        }
        else{
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
        Debug.Log("xz loss   "+loss.ToString());

        bool flag=true;
        //閾値は0.2くらい？(4-5の際の値が0.5532593のため)
        float threshold=0.004f;

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

            Debug.Log("High"+i.ToString()+" : "+yArray[i].ToString() +"Correct High"+i.ToString() + " : " + targetLength);
            loss+=1.0f*ConcurateHighLoss(yArray[i],targetLength);
            
            
        
            
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
    private float ConcurateDiameterLoss(float Diameter, float y,float DiameterBase){
        float loss=0.0f;

        float targetDiameter=ReturnCorrectDiameter(y,DiameterBase);

        Debug.Log("diameters"+" : "+Diameter.ToString()+" " +" Correct Diameters :" +targetDiameter.ToString() );

        loss=(float)Math.Pow((double)(targetDiameter-Diameter),2);

        return loss;
    }

    //直径の損失計算(すべて)
    private (float loss, bool flag) ConcurateAllDiameterLoss(float[] layersDiameter/*レイヤーごとの直径*/,float[] layersHigh,float DiameterBase){
        float loss=0.0f; //ロス
        bool flag=false; //あくまで念のためのフラグ

        

        //配列内のlossの計算
        for (int i=0; i<layersDiameter.Length;i++){
            
            loss += ConcurateDiameterLoss(layersDiameter[i],ReturnCorrectHigha(i,layersDiameter.Length),DiameterBase);
        }

        return (loss,flag);
    }
    
    private void parameterChanging(bool springFlag, float loss,int numLayer,int numPrism){
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
            Forces = new float[9];
            


            //パラメータの取得(edgeLoop)
            float[] edgeLoop;
            edgeLoop=new float[numLayer];
            for(int i=0;i<numLayer;i++){
                if(PlayerPrefs.HasKey("edgeLoop"+(i).ToString())){
                    edgeLoop[i]=PlayerPrefs.GetFloat("edgeLoop"+(i).ToString());
                }
            }

            //パラメータの取得(Strut basestrut)
            float[] strutScale;
            strutScale=new float[numLayer];
            for(int i=0;i<numLayer;i++){
                if(PlayerPrefs.HasKey("StrutScale"+(i).ToString())){
                    strutScale[i]=PlayerPrefs.GetFloat("StrutScale"+(i).ToString());
                }
            }
            

            //lossの保存
            PlayerPrefs.SetFloat("Loss"+compareFlag.ToString(),loss);
            PlayerPrefs.Save();

            //出力
            string savedata="";
            //念のため15くらいにしておこう
            for(int i=0;i<15;i++){
                //配列内なら配列を返し，配列外なら0を返す
                if(i<numLayer){
                    savedata+=strutScale[i].ToString()+",";
                }
                else{
                    savedata+="0,"; 
                }
            }
            
            //Debug.Log(tm_g_y);
            for(int i=0;i<15;i++){
                if(i<numLayer){
                    savedata+=edgeLoop[i].ToString()+",";
                }
                else{
                    savedata+="0,";
                }

                 
            }
            //savedata+=",highLoss,"+highLosses.ToString()+",highLoss,"+DiameterLoss[edgeFlag].ToString()+"\n";
            
            float sikiiti=0.02f*(float)numLayer;
            string filename=numLayer.ToString()+"_"+numPrism.ToString()+"_newTuype_1.5High";
            
            if(loss<=sikiiti){
                int photonumber=0;
                string picturePath="C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/picture/";
                if(PlayerPrefs.HasKey("photonumber")){
                    photonumber=PlayerPrefs.GetInt("photonumber");
                }
                if(System.IO.File.Exists(picturePath+filename)==false){
                    System.IO.Directory.CreateDirectory(picturePath+filename);
                }
                
                OnScrrenCapture(picturePath+filename+"/"+photonumber.ToString()+".png");
                
                savedata+=loss.ToString()+","+springForce.ToString()+","+compareFlag.ToString()+","+photonumber.ToString()+","+edgeFlag.ToString();
                photonumber+=1;
                PlayerPrefs.SetInt("photonumber",photonumber);
                PlayerPrefs.Save();
            }
            else{
                savedata+=loss.ToString()+","+springForce.ToString()+","+compareFlag.ToString()+",null,"+edgeFlag.ToString();
            }
            
            
            
            //Csvの出力
            


            //パラメータの調整
            float damperEdge=0.005f/numPrism;
            float damperStrut=0.0025f;
            int countUnchange=1;
            if(PlayerPrefs.HasKey("countUnchange")){
                countUnchange=PlayerPrefs.GetInt("countUnchange");
            }


            //一度も変化していない場合(0変化してない，1変化した)
            int UnchangFlag=0;
            if(PlayerPrefs.HasKey("changed")){
                UnchangFlag=PlayerPrefs.GetInt("changed");
            }

            switch(compareFlag){
                
                                    
                case 0://0 Strut+edgeLoop- 
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-damperEdge*((float)1/countUnchange);
                    
                    strutScale[edgeFlag]=strutScale[edgeFlag]+damperStrut*((float)1/countUnchange);
                    
                
                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    goto case -1;

                case 1://1 Strut+edgeLoop+ 
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+2.0f*damperEdge*((float)1/countUnchange);

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    goto case -1;

                case 2://2 Strut-edgeLoop-
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-2.0f * damperEdge*((float)1/countUnchange);
                    
                    
                    strutScale[edgeFlag]=strutScale[edgeFlag]-2.0f *damperStrut*((float)1/countUnchange);
                    
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    goto case -1;

                case 3://3 Strut-edgeLoop+ 
                    
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+2.0f*damperEdge*((float)1/countUnchange);
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    
                    goto case -1;
                

                case 4: //Strutそのままedge+
                    
                    
                    strutScale[edgeFlag]=strutScale[edgeFlag]+ damperStrut*((float)1/countUnchange);
                    
                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();
                    compareFlag=compareFlag+1;
                    
                    goto case -1;

                case 5: //Strutそのままedge-
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-2.0f*damperEdge*((float)1/countUnchange);

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();
                    
                    compareFlag=compareFlag+1;
                    goto case -1;

                case 6: //edgeそのままstrut+
                    edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+damperEdge*((float)1/countUnchange);
                    
                    
                    strutScale[edgeFlag]=strutScale[edgeFlag]+ damperStrut*((float)1/countUnchange);
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    goto case -1;

                case 7: //edgeそのままstrut-
                    
                    strutScale[edgeFlag]=strutScale[edgeFlag]-2.0f* damperStrut*((float)1/countUnchange);
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=compareFlag+1;
                    goto case -1;

                case 8://データの比較段階(時系列上3のデータがこの際とられている．)
                    //最後の加算が面倒なのでパラメータを最初に戻しておくついでに、前のデータとのloss比較する。
                    
                    
                    
                    strutScale[edgeFlag]=strutScale[edgeFlag]+ damperStrut*((float)1/countUnchange);
                    

                    PlayerPrefs.SetFloat("springForce"+compareFlag.ToString(),springForce);
                    PlayerPrefs.Save();

                    compareFlag=0;
                    goto case -1;

                case -1:
                    
                    Debug.Log("parameter adjusting...");
                    Debug.Log("countUnchange" + countUnchange.ToString());
                    break;
                

            }
            //Debug.Log(compareFlag);

            //lossの一番小さいやつ データ収集中は-1
            int mins=-1;

            //compareFlag+1されているため，最後のデータは=0の際に取得される。
            if(compareFlag==0){
                Debug.Log("----------------------- Compare phase -------------------------------");
                //以前のlossのデータを取得
                float[] losses;
                losses = new float[9];

                for(int i=0;i<9;i++){
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
                    //Debug.Log("losses  "+i.ToString()+"  " +losses[i].ToString());
                }


                //lossの比較
                float minloss=10000000.0f;
                
                //高さが良いやつを選抜する
     
                for(int i=0;i<9;i++){
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
                
                Debug.Log("mins  "+mins.ToString());
                
                

                //エラーをもとにパラメータの適用(case 3の際の値になっているので，そこから適用するとどうなる？)
                switch(mins){
                    
                    case 0://変更なし
                        springForce=Forces[8];
                        edgeFlag+=1;
                        UnchangFlag=0;
                        
                        break;


                    case 1://0 Strut+edgeLoop- 
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-damperEdge*((float)1/countUnchange);
                        
                        
                        strutScale[edgeFlag]=strutScale[edgeFlag]+damperStrut*((float)1/countUnchange);
                        
                        springForce=Forces[0];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;

                    case 2://1 Strut+edgeLoop+ 
                        
                       
                        strutScale[edgeFlag]=strutScale[edgeFlag]+damperStrut*((float)1/countUnchange);
                        
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+1.0f*damperEdge*((float)1/countUnchange);
                        springForce=Forces[1];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;

                    case 3://2 Strut-edgeLoop-
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-1.0f * damperEdge*((float)1/countUnchange);
                        
                        
                        strutScale[edgeFlag]=strutScale[edgeFlag]-damperStrut*((float)1/countUnchange);
                        
                        springForce=Forces[2];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;

                    case 4://3 Strut-edgeLoop+ 
                        
                        strutScale[edgeFlag]=strutScale[edgeFlag]-damperStrut*((float)1/countUnchange);
                        
                    
                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+damperEdge*((float)1/countUnchange);
                        springForce=Forces[3];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;

                    case 5://4 edgeLoop+ 

                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]+damperEdge*((float)1/countUnchange);
                        springForce=Forces[4];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;

                    case 6://5 edgeLoop-

                        edgeLoop[edgeFlag]=edgeLoop[edgeFlag]-damperEdge*((float)1/countUnchange);
                        springForce=Forces[5];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;

                    case 7://3 Strut+
                        
                        strutScale[edgeFlag]=strutScale[edgeFlag]+damperStrut*((float)1/countUnchange);
                        

                        springForce=Forces[6];
                        edgeFlag+=1;
                        UnchangFlag+=1;
                        break;
                    
                    case 8://3 Strut- 
                        
                        
                        strutScale[edgeFlag]=strutScale[edgeFlag]-damperStrut*((float)1/countUnchange);
                        

                        springForce=Forces[7];
                        edgeFlag+=1;
                        UnchangFlag+=1;
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
                
            }

            //edgeFlagのループさせる処理
            if(edgeFlag>=numLayer){
                edgeFlag=0;
                //一度もパラメータが変化しなかったときの調整
                if(UnchangFlag==0){
                    countUnchange+=1;
                }
                else if(UnchangFlag!=0){
                    countUnchange=1;
                }
            }
            PlayerPrefs.SetInt("UnchangFlag",UnchangFlag);
            PlayerPrefs.Save();
            PlayerPrefs.SetInt("countUnchange",countUnchange);
            PlayerPrefs.Save();
            
            

            //パラメータの引継ぎ
            for(int i=0;i<numLayer;i++){
                PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
                PlayerPrefs.Save();
                PlayerPrefs.SetFloat("StrutScale"+(i).ToString(),strutScale[i]);
                PlayerPrefs.Save();
            }

            Debug.Log("EdgeFlag : "+edgeFlag.ToString());
            
            PlayerPrefs.SetFloat("SpringForce",640.0f);//つぶれ始めるから，毎回リセットかければいいのでは？ #640.0ffだが，時間がかかりすぎる
            PlayerPrefs.Save();
            PlayerPrefs.SetInt("compareFlag",compareFlag);
            PlayerPrefs.Save();
            
            PlayerPrefs.SetInt("EdgeFlag",edgeFlag);
            PlayerPrefs.Save();

            //出力
            DateTime days =DateTime.Now;

            
            //フォルダがないなら作成とヘッダーの追加
            string folderpath="C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/"+numLayer.ToString()+"_"+numPrism.ToString();
            string path= folderpath+"/mountingSearc"+filename+".csv";
            if (Directory.Exists(folderpath)==false){
                Directory.CreateDirectory(folderpath);
                string header="";

                for(int i=1;i<=15;i++){
                    header+="strutScale"+i.ToString()+",";
                }

                for(int i=1;i<=15;i++){
                    header+="edgeloop"+i.ToString()+",";
                }
                header+="loss,springForce,compareFlag,photoNumber,edgeflag,min,date\n";
                OutputCsv(path,header);
            }
            savedata=savedata+","+mins.ToString()+","+days.ToString()+"\n";
            OutputCsv(path,savedata);

            Debug.Log(savedata);
            
            
            return;
        }
    }

    private (float sumEdges, float avgedGes) calcurateEdges(float[] LayerXpositions,float[] LayerYpositions,float[] LayerZpositions,int forNumber,int numLayer,int numPrism){

        float sumEdges=0.0f;
        float[,] positions;

        //手先位置のレイヤーの際
        if(forNumber==numLayer-1){
            positions=new float[numPrism,3];
            for(int i=0;i<numPrism;i++){
                //座標を2次元配列に格納 [レイヤー数,次元(x=0,y=1,z=2)]
                positions[i,0]=LayerXpositions[i];
                positions[i,1]=LayerYpositions[i];
                positions[i,2]=LayerZpositions[i];
            }
        }
        //手先位置以外のレイヤーの際
        else{
            positions=new float[numPrism*2,3];
            for (int i=0;i<numPrism*2;i=i+2){
                //座標を2次元配列に格納 [レイヤー数,次元(x=0,y=1,z=2)]
                positions[i,0]=LayerXpositions[i];
                positions[i,1]=LayerYpositions[i];
                positions[i,2]=LayerZpositions[i];

                positions[i+1,0]=LayerXpositions[i];
                positions[i+1,1]=LayerYpositions[i];
                positions[i+1,2]=LayerZpositions[i];
            }
        }

        float tm_g_x=0.0f;
        float tm_g_y=0.0f;
        float tm_g_z=0.0f;

        float avgedGes=0.0f;

        //edgeloopの差を計算
        for(int i=0;i<positions.GetLength(0)-1;i++){
            
            //XとYとZのそれぞれの距離
            tm_g_x=(float)Math.Pow((double)(positions[i,0]-positions[i+1,0]),2);
            tm_g_y=(float)Math.Pow((double)(positions[i,1]-positions[i+1,1]),2);
            tm_g_z=(float)Math.Pow((double)(positions[i,2]-positions[i+1,2]),2);

            sumEdges+=(float)Math.Sqrt(tm_g_x+tm_g_y+tm_g_z);
        }
        //手先位置
        if(forNumber==numLayer-1){
            //XとYとZのそれぞれの距離
            tm_g_x=(float)Math.Pow((double)(positions[numPrism-1,0]-positions[0,0]),2);
            tm_g_y=(float)Math.Pow((double)(positions[numPrism-1,1]-positions[0,1]),2);
            tm_g_z=(float)Math.Pow((double)(positions[numPrism-1,2]-positions[0,2]),2);
            sumEdges+=(float)Math.Sqrt(tm_g_x+tm_g_y+tm_g_z);
            avgedGes=sumEdges/(float)(numPrism*2);
        }
        //手先位置以外
        else{
            //XとYとZのそれぞれの距離
            tm_g_x=(float)Math.Pow((double)(positions[numPrism*2-1,0]-positions[0,0]),2);
            tm_g_y=(float)Math.Pow((double)(positions[numPrism*2-1,1]-positions[0,1]),2);
            tm_g_z=(float)Math.Pow((double)(positions[numPrism*2-1,2]-positions[0,2]),2);
            sumEdges+=(float)Math.Sqrt(tm_g_x+tm_g_y+tm_g_z);
            avgedGes=sumEdges/(float)numPrism;
        }
        

        
        
        
        return (sumEdges,avgedGes); 
    }

    // Start is called before the first frame update
    void Start()
    {           
        //StartCoroutine(DelayMethod());
    }

    // Update is called once per frame
    void Update()
    {   ///*
        if(Time.realtimeSinceStartup<30.0f){
            StartCoroutine(getsHandsPositions());
            counts++;
        }  
        //*/

        //show_handposition();
    }

    IEnumerator getsHandsPositions(){
        getHandPositions("tm_g_1");
        getHandPositions("tm_g_2");
        getHandPositions("tm_g_3");

        //0.05秒の待機
        yield return new WaitForSeconds(0.03f);
        
    } 

    void getHandPositions(string ManipulatoName){
        //Asembly.csの取得
        GameObject tm_g =GameObject.Find(ManipulatoName);
        Assembly assembly;
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);
        int numLayer=assembly.numLayer;
        int numPrism=assembly.numPrism;

        //0から計測のため，numLayer-1となる．
        var positionRetrun = getPosition(numLayer-1,ManipulatoName);

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

        float timeNow=Time.realtimeSinceStartup;
        
        string path="C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/20260206_handPosition_layer_"+numLayer.ToString()+"_prism_"+numPrism.ToString()+".csv";
        if(counts==0){
            //データの最初のラベル配置
            string label="time,x,y,z\n";
            OutputCsv(path,label);
        }
        string savedata=timeNow.ToString()+","+xAverage+","+yAverage+","+zAverage+"\n";
        if(counts%20==0){
            string picturePath="C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/picture/hand/handPosition_layer_"+numLayer.ToString()+"_prism_"+numPrism.ToString()+"_"+timeNow.ToString()+".png";
            OnScrrenCapture(picturePath);
        }
        

        
        OutputCsv(path,savedata);
        Debug.Log(savedata);

        
    }

    IEnumerator DelayMethod(){

        //Asembly.csの取得
        GameObject tm_g =GameObject.Find("tm_g_1");
        Assembly assembly;
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);

        //レイヤー数の取得
        int numLayer=assembly.numLayer;
        int numPrism=assembly.numPrism;
        float DiameterBase=assembly.radiusBase*2.0f;

        //手先位置の取得変数
        float tm_g_x=0.0f;
        float tm_g_y=0.0f;
        float tm_g_z=0.0f;
        float[] tmpos =new float[3];

        //高さの取得
        float[] tm_g_layers_y;
        tm_g_layers_y = new float[numLayer];


        

        yield return new WaitForSeconds(3.55f);


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

        float[] DiameterLosses;
        DiameterLosses = new float [numLayer];

        //層ごとの高さ，直径，ズレの検出
        for(int i=0;i<numLayer;i++){

            //座標取得
            var positionRetrun = getPosition(i,"tm_g_1");
            
            //配列の長さの確認
            arrayLength=positionRetrun.arrayLength;
            //座標格納
            LayerXpositions=positionRetrun.LayerXpositions;
            LayerYpositions=positionRetrun.LayerYpositions;
            LayerZpositions=positionRetrun.LayerZpositions;

            var cals=calcurateEdges(LayerXpositions,LayerYpositions,LayerZpositions,i,numLayer,numPrism);

            Debug.Log("Layer"+i.ToString()+","+  " Avg : "+cals.avgedGes.ToString()+"       sum : "+cals.sumEdges.ToString());

        
            //配列にそれぞれの直径を格納
            diameters[i]=getDiameter(arrayLength,LayerXpositions,LayerZpositions);
            maxDistance=0;


            //配列にそれぞれの層の高さを格納
            tm_g_layers_y[i]=getHigh(LayerYpositions,arrayLength);
            

            //配列にそれぞれの層のずれ(end同士の高さの差)を格納
            maxErrors[i]=getDeviation(LayerYpositions,arrayLength);


            

            //ReturnCorrectHigha(i,numLayer);

            
             
           
        }
        
        //層ごとの半径のロス
        var DiameterData=ConcurateAllDiameterLoss(diameters,tm_g_layers_y,DiameterBase);
        //Debug.Log("Diameter Loss"+i.ToString() +"  "+DiameterData.loss.ToString());

        //層ごとの高さのロス
        var highLosses = ConcurateAllHighLoss(tm_g_layers_y);

        //手先位置(一番上の高さのロス)
        //float handPositionLoss=(float)(numLayer*numPrism/*重み*/)*ConcurateHighLoss(tm_g_layers_y[numLayer-1],1.48762f);

        //手先位置(一番上の高さのロス)
        float handPositionLoss=ConcurateHighLoss(tm_g_layers_y[numLayer-1],1.5f);


        //倒れているか確認
        var springlosses = ConculateSpringsLoss(LayerXpositions,LayerZpositions);

        //層ごと高さと直径のlossの合成
        float allLoss=highLosses.loss+DiameterData.loss+handPositionLoss;

        Debug.Log("hand loss" + handPositionLoss.ToString());
        Debug.Log("high loss" + highLosses.loss.ToString());
        Debug.Log("diameter loss" + DiameterData.loss.ToString());
        //float allLoss=0.8f*highLosses.loss+1.25f*DiameterData.loss;
        //float allLoss=DiameterData.loss+highLosses.loss;

        
        //Debug.Log(allLoss);
        
        //パラメータの変更処理
        parameterChanging(springlosses.flag,allLoss,numLayer,numPrism);

        int photonumber=0;

        

        
        
        if(PlayerPrefs.HasKey("photonumber")){
            photonumber=PlayerPrefs.GetInt("photonumber");
        }
        if(photonumber<=5000*numLayer){
            SceneManager.LoadScene("SampleScene");
        }
        

    }

    
    void show_handposition(){
        //Asembly.csの取得
        GameObject tm_g =GameObject.Find("tm_g_1");
        Assembly assembly;
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);

        //レイヤー数の取得
        int numLayer=assembly.numLayer;
        int numPrism=assembly.numPrism;
        float DiameterBase=assembly.radiusBase*2.0f;

        //手先位置の取得変数
        float tm_g_x=0.0f;
        float tm_g_y=0.0f;
        float tm_g_z=0.0f;
        float[] tmpos =new float[3];

        //x,y,z座標の取得用配列
        float[] LayerXpositions;
        float[] LayerYpositions;
        float[] LayerZpositions;


        //配列長さはレイヤーが上下の柱より構成されるため2倍
        LayerXpositions= new float [numPrism*2];
        LayerYpositions= new float [numPrism*2];
        LayerZpositions= new float [numPrism*2]; 

        var positionRetrun = getPosition(numLayer-1,"tm_g_1");

        
            
        //配列の長さの確認
        int arrayLength=0;
        arrayLength=positionRetrun.arrayLength;
        //座標格納
        LayerXpositions=positionRetrun.LayerXpositions;
        LayerYpositions=positionRetrun.LayerYpositions;
        LayerZpositions=positionRetrun.LayerZpositions;

        Debug.Log(getHigh(LayerYpositions,arrayLength));



    }
    
}
 