using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

using UnityEngine.SceneManagement;

public class Searcher2 : MonoBehaviour
{
    

    private GameObject[] struts;
    private GameObject[] end;
    private float[] position_x;
    private float[] position_y;
    private float[] position_z;
    private float[] y_s;

    Assembly assembly;
    
    private float toggle =0.1f;
    private int count=0;

    public float[] trueRadius={0.28710955769095553f,0.25047506156241606f,0.212640851049945f,0.18240953443903773f,0.17003731799445732f};//4-5の時の半径 {0.28710955769095553f,0.25047506156241606f,0.212640851049945f,0.18240953443903773f,0.17003731799445732f}
    private float targetLength = 1.522883f;
    private float[] targetRadius;
    private int numLayer;
    private int numPrism;
    private float radiusBase;

    // Start is called before the first frame update
    void Start()
    {   
        
        StartCoroutine(DelayMethod());
 

    }

    // Update is called once per frame
    void Update()
    {
        //None;
        
        
    }
    
    //スクショ関数
    void OnScrrenCapture(string path){
        ScreenCapture.CaptureScreenshot(path);
    }

    void OutputCsv(string path,string savedata ){

        //File.AppendAllText("C:/Users/Yamauchi Gaito/Desktop/workspace/tmsim/data/data.csv",savedata);
        File.AppendAllText(path,savedata);
    }

    private float Computeloss(float[] radius,float[] targetRadius,float[] position,float[] middle){
        float loss=0.0f;
        float lengthweight=1.0f;
        float xweight=1.0f;
        float middleweight=1.0f;
        float radiusweight=1.0f;

        float highLoss=ComputeHighloss(position[1]);
        float XZLoss= ComputeXZLoss(position[0],position[2]);
        float middleLoss=ComputeMiddleLoss(middle[0],middle[1]);
        float radiusLoss=ComputeRadiusLoss(radius,targetRadius);

        Debug.Log(radiusLoss);

        loss=lengthweight*highLoss+xweight*XZLoss+middleweight*middleLoss+radiusweight*radiusLoss;
        
        return loss;
    } 

    private float ComputeHighloss(float tensegrityHith){
        float loss=0.0f;
        //loss=(float)(Math.Pow((double)(targetLength-tensegrityHith),2)); 
        loss=(tensegrityHith-targetLength); 

        if(loss<0){
            loss=(targetLength-tensegrityHith);
            //loss=(float)(Math.Pow((double)(targetLength-tensegrityHith),2)); 
            //loss=loss*10;
        }

        //Debug.Log(tensegrityHith);
           
        Debug.Log("high loss"+loss.ToString());
        return loss;
    }

    private float ComputeXZLoss(float x, float z){
        float loss=0.0f;
        loss=((float)(Math.Pow((double)(0-x),2))+(float)(Math.Pow((double)(0-z),2)));
        Debug.Log("xz loss"+loss.ToString());
        return loss;
    }

    private float ComputeMiddleLoss(float x, float z){
        float loss=0.0f;
        loss=((float)(Math.Pow((double)(0-x),2))+(float)(Math.Pow((double)(0-z),2)));
        Debug.Log("middle loss"+loss.ToString());
        return loss;
    }

    private float ComputeRadiusLoss(float[] radius,float[] targetRadius){
        float loss=0.0f;
        for(int i=0;i<numLayer;i++){
            loss=loss+(float)(Math.Pow((double)(targetRadius[i]-radius[i]),2));
        }
        Debug.Log("radius loss"+loss.ToString());
        return loss;
    }
    

    IEnumerator DelayMethod(){


        int parameterFlag=0;
        int compareFlag=0;
        
        if(PlayerPrefs.HasKey("parameterFlag")){
            parameterFlag=PlayerPrefs.GetInt("parameterFlag");
        }
        if(PlayerPrefs.HasKey("compareFlag")){
            compareFlag=PlayerPrefs.GetInt("compareFlag");
        }

        //目標値(4本,5層の際の半径)(手動で入力)
        

        //Asembly.csの取得
        GameObject tm_g =GameObject.Find("tm_g");
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);

        //レイヤー数の取得
        numLayer=assembly.numLayer;
        numPrism=assembly.numPrism;
        radiusBase=assembly.radiusBase;

        //edgeの長さ調整
        
        position_x =new float[numLayer];
        position_y =new float[numLayer];
        position_z =new float[numLayer];
        
        struts = new GameObject[numLayer];
        end = new GameObject[numLayer];

        //手先位置の取得変数
        float tm_g_x=0.0f;
        float tm_g_y=0.0f;
        float tm_g_z=0.0f;
        float[] tmpos =new float[3];



        //目標値の線形補完
        float[] targetRadius = new float[numLayer]; 
         
        //5層の倍数の場合の処理
        if(numLayer%(trueRadius.Length)==0){
            //補完範囲の分割数
            int segment = numLayer/trueRadius.Length;
            for(int i=0;i<numLayer;i++){
                if(i%segment==0){
                    //値が決まっているところへtrueRadiusの代入
                    targetRadius[i]=trueRadius[i/segment];
                }
                else{
                    //決まっていないところはありえない値を入れて初期化
                    targetRadius[i]=1110.0f;
                }
            }
            //間を求める勾配
            float gradient=0.0f;
            //現在の分割数内の値
            int countSegment=1;
            for(int i=0;i<numLayer;i++){
                //ありえない値で初期化されていた際に目標値を設定する．
                if(targetRadius[i]==1110.0f){
       
                    targetRadius[i]=gradient*(float)(i%countSegment+1)+targetRadius[i-countSegment];
                    countSegment+=1;
                }
                //値が決まっているなら，勾配の変更
                else if(i!=(numLayer-segment)){//(nullPointerにならないように最後のみこの処理を行わない)
                    gradient=(targetRadius[i+segment]-targetRadius[i])/segment;
                    countSegment=1;
                }
                //Debug.Log(targetRadius[i]);
            }
        }
        //5層の倍数ではない際の処理
        else{
            float gradient=0.0f;
            //上から下まで一次関数と仮定して勾配を求める
            gradient=-(trueRadius[0]-trueRadius[5])/numLayer;
            for(int i=0;i<=numLayer;i++){
                targetRadius[i]=trueRadius[0]-gradient*(float)i;
            }
        }


         

        
        ////////////////////////////////////////////////////////////////////////////////////////



        //登録情報の保存
        PlayerPrefs.Save();
        ////////////////////////////////////////////////////////////////////////////////////////

        //時間の停止
        yield return new WaitForSeconds(1.55f);
                

        //高さの取得処理
        ////////////////////////////////////////////////////////////////////////////////////////
        string num1;
        string num2="Strut7";

        
        y_s =new float[8];

        for(int i=2;i<10;i++){
            num1=i.ToString();
            num1=num2+num1;
            struts[i-2] = tm_g.transform.Find(num1).gameObject;
        }
        //座標集計
        for(int i=0;i<8;i++){
            end[i]=struts[i].transform.Find("end1").gameObject;
            position_x[i]=end[i].transform.position.x;
            position_y[i]=end[i].transform.position.y;
            position_z[i]=end[i].transform.position.z;
            
        }

        for(int i=0;i<8;i++){
            tm_g_x+=position_x[i];
            tm_g_y+=position_y[i];
            tm_g_z+=position_z[i];
        }

        var a=tm_g.transform.Find("Strut0").gameObject;
        var b=a.transform.Find("end").gameObject;
        y_s[0]=b.transform.position.y;

        for(int j=1;j<8;j++){
            a=tm_g.transform.Find("Strut"+j.ToString()+0).gameObject;
            b=a.transform.Find("end1").gameObject;
            y_s[j]=b.transform.position.y;
        }
        

        //手先位置の計測のため，上の点の値取得
        tmpos[0]=tm_g_x/8;
        tmpos[1]=tm_g_y/8;
        tmpos[2]=tm_g_z/8;

        float strutscale = assembly.StrutScale;
        ////////////////////////////////////////////////////////////////////////////////////////

        

        //半径の集計
        ////////////////////////////////////////////////////////////////////////////////////////
        

        float[] radius=new float[numLayer];
        //マニピュレータの中心点
        float[] middle=new float[2];
        //層の数だけループ

        //ベースストラットのみendという 名前であるため，例外処理
        var underStrut=tm_g.transform.Find("Strut0").gameObject;
        var end1=underStrut.transform.Find("end").gameObject;
        var currentStrut=tm_g.transform.Find("Strut"+numPrism.ToString()).gameObject;
        var end2=currentStrut.transform.Find("end2").gameObject;
        //radius[0]=(float)Math.Sqrt(Math.Pow((double)(end1.transform.position.x-end2.transform.position.x),2)+Math.Pow((double)(end1.transform.position.z-end2.transform.position.z),2));    //x^2+z^2



 
        for(int k=0;k<numLayer-1;k++){
            underStrut=tm_g.transform.Find("Strut"+(numPrism*k).ToString()).gameObject;//隣り合うストラットとの数字の関係は+numPrismずつ増えていく(8,10の際)
            if(k==0){
                end1=underStrut.transform.Find("end").gameObject;
            }
            else{
                end1=underStrut.transform.Find("end1").gameObject;
            }
            currentStrut=tm_g.transform.Find("Strut"+(numPrism*(k+1)).ToString()).gameObject;
            end2=currentStrut.transform.Find("end2").gameObject;
            radius[k]=((float)Math.Sqrt(Math.Pow((double)(end1.transform.position.x-end2.transform.position.x),2)+Math.Pow((double)(end1.transform.position.z-end2.transform.position.z),2)))/(2.0f*(float)Math.Sin(Math.PI/(2*numPrism))); 
            

            if(numLayer%2==0){
                if(k==numLayer/2){
                    underStrut=tm_g.transform.Find("Strut"+(numPrism*k).ToString()).gameObject;//一番遠いストラットとの数字の関係は+numPrism/2ずつ増えていく(8,10の際)
                    end1=underStrut.transform.Find("end1").gameObject;
                    currentStrut=tm_g.transform.Find("Strut"+(numPrism*(k)+(numPrism/2)).ToString()).gameObject;
                    end2=underStrut.transform.Find("end1").gameObject;
                    middle[0]=end1.transform.position.x+end2.transform.position.x; 
                    middle[1]=end1.transform.position.z+end2.transform.position.z; 
                    //Debug.Log("Strut"+(numPrism*k).ToString());
                    //Debug.Log("Strut"+(numPrism*(k)+(numPrism/2)));
                    //Debug.Log(middle[0]);
                    //Debug.Log(middle[1]);
                }
                else if(k==(numLayer/2)-1){

                }
            }
            
        }
        //最後の層である手先位置においては隣り合うストラットは±1であり，どちらもend1を用いる(これより上の層は存在しないため．)
        underStrut=tm_g.transform.Find("Strut"+(numPrism*(numLayer-1)).ToString()).gameObject;//隣り合うストラットとの数字の関係は+numPrismずつ増えていく(8,10の際)
        end1=underStrut.transform.Find("end1").gameObject;
        currentStrut=tm_g.transform.Find("Strut"+(numPrism*(numLayer-1)+1).ToString()).gameObject;
        end2=currentStrut.transform.Find("end1").gameObject;
        radius[numLayer-1]=(float)Math.Sqrt(Math.Pow((double)(end1.transform.position.x-end2.transform.position.x),2)+Math.Pow((double)(end1.transform.position.z-end2.transform.position.z),2)); 

        ////////////////////////////////////////////////////////////////////////////////////////
                
        //次の周回のstrut長さ変数の変更
        float baseScale=assembly.BaseScale;
        float strutScale=assembly.StrutScale;
        float[] edgeLoop =assembly.edgeLoop;


        float damperStrut=0.02f;
        float damperDamperedge=0.000001f;
        float damperedge=0.0f;
        
        
        //次の週へのパラメータの変更
        switch(parameterFlag){
            case -2:
                //baseStrutのパラメータ調整
                if(compareFlag==0){
                    baseScale-=damperStrut;

                    PlayerPrefs.SetFloat(("EndEffecterX"),tmpos[0]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat(("EndEffecterY"),tmpos[1]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat(("EndEffecterZ"),tmpos[2]);
                    PlayerPrefs.Save();

                    PlayerPrefs.SetFloat("middle_x",middle[0]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat("middle_z",middle[1]);
                    PlayerPrefs.Save();

                    for(int i=0;i<radius.Length;i++){
                        PlayerPrefs.SetFloat(("ChangedRadius"+i.ToString()),radius[i]);
                        PlayerPrefs.Save();
                    }
                }
                else if(compareFlag==1){
                    baseScale+=2.0f*damperStrut;
                    parameterFlag+=1;
                }
                compareFlag+=1;
                Debug.Log("baseScale"+baseScale.ToString());
 
                break;
            case -1:
                //strutの調整
                if(compareFlag==0){
                    strutScale-=damperStrut;

                    PlayerPrefs.SetFloat(("EndEffecterX"),tmpos[0]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat(("EndEffecterY"),tmpos[1]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat(("EndEffecterZ"),tmpos[2]);
                    PlayerPrefs.Save();

                    PlayerPrefs.SetFloat("middle_x",middle[0]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat("middle_z",middle[1]);
                    PlayerPrefs.Save();

                    for(int i=0;i<radius.Length;i++){
                        PlayerPrefs.SetFloat(("ChangedRadius"+i.ToString()),radius[i]);
                        PlayerPrefs.Save();
                    }
                }
                else if(compareFlag==1){
                    strutScale+=2.0f*damperStrut;
                    parameterFlag+=1;
                }
                compareFlag+=1;
                Debug.Log("strutScale"+strutScale.ToString());

                break;
            default:
                //エッジループの調整
                if(compareFlag==0){
                    damperedge=-damperDamperedge*((float)((numLayer-1)/numLayer)*(float)(parameterFlag+1)+(float)numLayer);
                    Debug.Log("damperedge  "+damperedge.ToString());

                    edgeLoop[parameterFlag]=edgeLoop[parameterFlag]-damperedge;
                    Debug.Log("edgeLoop"+parameterFlag.ToString()+"          "+edgeLoop[parameterFlag].ToString());

                    PlayerPrefs.SetFloat(("EndEffecterX"),tmpos[0]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat(("EndEffecterY"),tmpos[1]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat(("EndEffecterZ"),tmpos[2]);

                    PlayerPrefs.SetFloat("middle_x",middle[0]);
                    PlayerPrefs.Save();
                    PlayerPrefs.SetFloat("middle_z",middle[1]);
                    PlayerPrefs.Save();
                    PlayerPrefs.Save();

                    for(int i=0;i<radius.Length;i++){
                        PlayerPrefs.SetFloat(("ChangedRadius"+i.ToString()),radius[i]);
                        PlayerPrefs.Save();
                    }
                }
                else if(compareFlag==1){
                    edgeLoop[parameterFlag]=edgeLoop[parameterFlag]+2.0f*damperedge;
                    Debug.Log("edgeLoop"+parameterFlag.ToString()+"          "+edgeLoop[parameterFlag].ToString());
                    if(parameterFlag==numLayer-1){
                    //パラメータflagを-2にしてstrutの調整へ
                        parameterFlag=-2;
                    }
                    else{
                        parameterFlag+=1;
                    }
                }
                compareFlag+=1;
                
                
                break;
        }
        
        
        

        //山登り法による引き渡すべきパラメータの決定

        float errorMinus=0.0f;
        float errorPlus=0.0f;
        float[] ChangedPos=new float[3];
        float[] ChangeMiddle=new float[2];
        float[] ChangedRadius= new float[radius.Length];
        float error=0.0f;
        //float differror=100000.0f;
        string errorName="error";

        if(PlayerPrefs.HasKey("differror")){
            //differror=PlayerPrefs.GetFloat("differror");
        }

        //出力
        string debugdata="basescale,"+baseScale.ToString()+",strutScale,"+strutScale.ToString();
        //Debug.Log(tm_g_y);
        for(int i=0;i<numLayer;i++){
            debugdata+=",edgeLoop"+i.ToString() +"," +edgeLoop[i].ToString(); 
        }

        Debug.Log(debugdata);
        


        //山のぼり法による最適化
        if(compareFlag==2){
            //変化前のやつを取得
            for(int j=0;j<radius.Length;j++){
                ChangedRadius[j]=PlayerPrefs.GetFloat("ChangedRadius"+(j).ToString());
            }
            ChangedPos[0]=PlayerPrefs.GetFloat("EndEffecterX");
            ChangedPos[1]=PlayerPrefs.GetFloat("EndEffecterY");
            ChangedPos[2]=PlayerPrefs.GetFloat("EndEffecterZ");
            ChangeMiddle[0]=PlayerPrefs.GetFloat("middle_x",middle[0]);
            ChangeMiddle[1]=PlayerPrefs.GetFloat("middle_z",middle[1]);
            
            switch(parameterFlag-1){
                case -3://parameterflagがnumLayer-1の次の処理おかしくなるということで追加
                    errorMinus=Computeloss(ChangedRadius,targetRadius,ChangedPos, ChangeMiddle);
                    errorPlus=Computeloss(radius,targetRadius,tmpos,middle);
                    
                    if(errorPlus<errorMinus){
                        error=errorPlus;
                        errorName="errorPlus";
                    }
                    else{
                        damperedge=-damperDamperedge*((float)((numLayer-1)/numLayer)*(float)(numLayer)+(float)numLayer);
                        edgeLoop[numLayer-1]=edgeLoop[numLayer-1]-2.0f*damperedge;
                        error=errorMinus;
                        errorName="errorMinus";
                    }
                    
                    Debug.Log("errortype    :" +errorName);
                    Debug.Log("edgeloop"+(numLayer-1).ToString()+"          "+edgeLoop[numLayer-1].ToString());
                   break;
                case -2:
                    //diff(二つ)と今のものを最小二乗法で比較(高さ)
                    errorMinus=Computeloss(ChangedRadius,targetRadius,ChangedPos, ChangeMiddle);
                    errorPlus=Computeloss(radius,targetRadius,tmpos,middle);
                    //
                    Debug.Log(errorPlus);
                    Debug.Log(errorMinus);
                   
                    if(errorPlus<errorMinus){
                        error=errorPlus;
                        errorName="errorPlus";
                        //現在のBaseのStrutScaleの保存
                    }
                    else{
                        baseScale-=2.0f*damperStrut;
                        error=errorMinus;
                        errorName="errorMinus";
                        //現在のBaseのStrutScaleの保存
                    }
                    Debug.Log("errortype    :" +errorName);
                    Debug.Log("baseScale    :" +baseScale.ToString());
                    break;
                case -1:
                    //diff(二つ)と今のものを最小二乗法で比較(高さ)
                    errorMinus=Computeloss(ChangedRadius,targetRadius,ChangedPos, ChangeMiddle);
                    errorPlus=Computeloss(radius,targetRadius,tmpos,middle);
                    
                    if(errorPlus<errorMinus){
                        error=errorPlus;
                        errorName="errorPlus";
                    }
                    else{
                        strutScale-=2.0f*damperStrut;
                        error=errorMinus;
                        errorName="errorMinus";
                    }
                    Debug.Log("errortype    :" +errorName);
                    Debug.Log("strutScale    :" +strutScale.ToString());
                    break;
                default:
                    //diff(二つ)と今のものを最小二乗法で比較(半径)
                    errorMinus=Computeloss(ChangedRadius,targetRadius,ChangedPos, ChangeMiddle);
                    errorPlus=Computeloss(radius,targetRadius,tmpos,middle);
                    
                    if(errorPlus<errorMinus){
                        
                        error=errorPlus;
                        errorName="errorPlus";
                    }
                    else{
                        damperedge=-damperDamperedge*((float)((numLayer-1)/numLayer)*(float)(parameterFlag)+(float)numLayer);
                        edgeLoop[parameterFlag-1]=edgeLoop[parameterFlag]-2.0f*damperedge;
                        error=errorMinus;
                        errorName="errorMinus";
                    }
                    Debug.Log("errortype    :" +errorName);
                    Debug.Log("edgeloop"+(parameterFlag-1).ToString()+"          "+edgeLoop[parameterFlag-1].ToString());
                    break;

            }
            
            //Debug.Log(errorMinus);
            //Debug.Log(errorPlus);

            //制約条件
            if(baseScale>=1.0f){
                baseScale=0.99f;
                Debug.Log("=====================================basescaleに制約条件が課せられました．=====================================");
            }
            else if(baseScale<=0.0f){
                baseScale=0.2f;
                Debug.Log("=====================================basescaleに制約条件が課せられました．=====================================");
            }
            if(strutScale>=1.0f){
                strutScale=0.99f;
                Debug.Log("=====================================strutScaleに制約条件が課せられました．=====================================");
            }
            else if(strutScale<=0.0f){
                strutScale=0.2f;
                Debug.Log("=====================================strutScaleに制約条件が課せられました．=====================================");
            }
            
            //(edgeloop_0<edgeloop_i<edgeLoop_{numLayer-1})
            float edgeLoop_minuis1=2.0f*radiusBase*(float)Math.Sin(Math.PI/(2*numPrism));//edgeloop0の最小値としてベースにエッジループがあると仮定
            
            
            if(edgeLoop[numLayer-1]<0){
                damperedge=-damperDamperedge*((float)((numLayer-1)/numLayer)*(float)(numLayer)+(float)numLayer);
                edgeLoop[numLayer-1]=damperedge*2;
                Debug.Log("=====================================edgeLoop9に制約条件が課せられました．=====================================");
            }
            /*
            for(int j=1; j<numLayer-2; j++){
                //Debug.Log((numLayer-j-1));
                if(edgeLoop[numLayer-j-1]>edgeLoop[numLayer-j]){
                    
                    edgeLoop[numLayer-j-1]=edgeLoop[numLayer-j]+damperDamperedge*2.0f;
                    Debug.Log("=====================================edgeLoop"+j.ToString()+"に制約条件が課せられました．=====================================");
                }
                if(edgeLoop[numLayer-j-1]<0){
                    edgeLoop[numLayer-j-1]=damperDamperedge*2.0f;
                    Debug.Log("=====================================edgeLoop"+j.ToString()+"に制約条件が課せられました．=====================================");
                }
            }
            */
            
            for(int j=1; j<numLayer-2; j++){
                damperedge=-damperDamperedge*((float)((numLayer-1)/numLayer)*(j)+numLayer);
                if(edgeLoop[j]>edgeLoop[j-1]){
                    edgeLoop[j]=edgeLoop[j-1]-damperedge*2.0f;
                    Debug.Log("=====================================edgeLoop"+j.ToString()+"に制約条件が課せられました．=====================================");
                }
                if(edgeLoop[j]<0){
                    edgeLoop[j]=damperedge*2;
                    Debug.Log("=====================================edgeLoop"+j.ToString()+"に制約条件が課せられました．=====================================");
                }
            }
            

            if(edgeLoop[0]<0 || edgeLoop_minuis1<edgeLoop[0]){
                damperedge=-damperDamperedge*((float)((numLayer-1)/numLayer)*(0.0f)+(float)numLayer);
                edgeLoop[0]=edgeLoop_minuis1-damperedge*2.0f;
                Debug.Log("=====================================edgeLoop0に制約条件が課せられました．=====================================");
            }

            

            

            //出力
            string savedata="basescale,"+baseScale.ToString()+",strutScale,"+strutScale.ToString();
            //Debug.Log(tm_g_y);
            for(int i=0;i<numLayer;i++){
                savedata+=",edgeLoop"+i.ToString() +"," +edgeLoop[i].ToString(); 
            }
            savedata+=",error,"+error.ToString()+",errorName,"+errorName+",parameterflag,"+(parameterFlag-1).ToString()+"\n";

            Debug.Log(savedata);
            //Csvの出力
            string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/mountingSearch.csv";
            OutputCsv(path,savedata);
        }   

        

        
        
        if(compareFlag>=2){
            compareFlag=0;
        }

        //tm_g_yの保存
        //radiusの保存
                
        //値変更の引継ぎ
        ////////////////////////////////////////////////////////////////////////////////////////    

        for(int i=0;i<edgeLoop.Length;i++){
            PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
            PlayerPrefs.Save();
        }
        PlayerPrefs.SetFloat("differror",error);
        PlayerPrefs.Save();
        PlayerPrefs.SetFloat("baseScale",baseScale);
        PlayerPrefs.Save();
        PlayerPrefs.SetFloat("strutScale",strutScale);
        PlayerPrefs.Save();
        PlayerPrefs.SetInt("compareFlag",compareFlag);
        PlayerPrefs.Save();
        PlayerPrefs.SetInt("parameterFlag",parameterFlag);
        PlayerPrefs.Save();
        
        ////////////////////////////////////////////////////////////////////////////////////////

        //シーンの遷移
        SceneManager.LoadScene("SampleScene");
        
        
    }

    

        
}

