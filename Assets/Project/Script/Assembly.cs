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

[SerializeField]
public class Assembly : MonoBehaviour
{
    // Prefabs
    //================================================
    // Expected parameters
    // basePlate: base
    // baseStrut: base_strut
    // middleStrut: strut
    // loopLine: loop_renderer
    // verticalLine: stripe_renderer
    //================================================
    public GameObject basePlate;
    public GameObject baseStrut;
    public GameObject middleStrut;
    public GameObject loopLine;
    public GameObject vertcalLine;

    // Public Member Variables
    //================================================
    // Recommended parameters for TM40
    // numLayer = 5                                         //層の数
    // numPrism = 4                                         //ストラットの数
    // radiusBase = 0.22                                    //
    // edgeLoop = {0.05, 0.04, 0.03, 0.02, 0.05,0.01}            //配列の一つの数値ごとに 各層に対応している
    // *) The length of loopEdge has to be equal to numLayer, 
    //    otherwise it will be ignored.
    //================================================
    public int numLayer;            //層の数
    public int numPrism;            //ストラットの数
    //public float radius;        
    public float radiusBase;        //アームの土台の半径(アームの土台同士の幅の決定に用いる)
    public float[] edgeLoop;        //ばねの長さのMaxの設定
    public bool isPausedStart;
    public bool isNeverSleep;
    public bool springCollision;    //ばねの当たり判定の有無
    public float stripeshape;       //r-Θのr部分（柱の点作るときの参照値）
    //public float radiusLoop=0.0353f;                                         //edgeloopの中心からの距離
    public float radiusLoop;

    //ばね定数の設定
    public float springForce;                                         //ばねの強さ
    public float SpringDmaper;                                                 //層に比例するばねの強さ
    public bool earthConnected;

    public float[] StrutScale;

    //edgeの計算方法の選択
    public bool log_x;              //edgeloopの計算にLog(x)を扱うか    (底数が10)
    public bool ln_x;               //edgeloopの計算にLn(x)を扱うか     (底数がe)
    public bool sqrt_x;             //edgeloopの計算にsqrt(x)を扱うか

    public bool difAndRatio;        //edgeloopの計算にエッジの長さの引き算を扱うか

    public bool edgeReverse;        //edgeの数字を入れ替え

    public float edgeDampoer;       //Defaultedgeを1とした際のDefaultedgeLastとの比
    public float edgeparameter_layers=1.0f;  //レイヤー数とかで変わる減衰率

    public float predict;
    public double psi0;
    public double psiFin;
    public double phi0;
    public double phiFin;

    public bool exportBotton;

    //エッジループとかで固定値扱うか
    public bool useConstant;
    //関数使うか？
    public bool useFunc;



    //ここに変数をpiblicで設定してinspectorで調整できるものを増やそう．
    //柱の長さと紐を触れるようにしよう．(ばね定数が強かったり紐が短いとへこむことがある．)

    // Array to keep struts
    //================================================
    // Because this array holds almost all important info,
    // it sets this to public but keeps hidden for the inspector.
    // For instance, you acess this array from other script 
    // like this:
    //-----------------------------------------
    // //<<As class member:>>
    // public GameObject tmObject;
    // //<<After assigning the tm40 prefab's instance to tmObject>>
    // Assembly tm = tm.GetComponent<Assembly>;
    // // you can access to the i-th strut as tm.struts[i]
    //================================================
    [HideInInspector]
    public GameObject[] struts;

    // Array to keep other instances of prefabs.
    private GameObject[] baseblocks;
    private GameObject[] loops;
    private GameObject[] stripes;
    private Transform[] structsEnd1;
    private Transform[] structsEnd2;
    
    // Flag that all instances are generated
    [HideInInspector]
    public bool isReady = false;

    // Generate the strut number at the j-th strut in the i-th layer
    public int index(int i, int j) //structの名前生成用の数字制作
    {
        if (j < 0) {
            j = j + numPrism;                           //ｊが0より小さいと，柱の数を足す
        } else if (j >= numPrism) {
            j = j - numPrism;                           //jが柱の数より大きいとjより柱の数を引く
        }
        
        if (j < 0 || j >= numPrism){
            return -1;                                  //jが0より小さいか，柱の数より大きいと-1を返す(上の処理の例外時?)
        } else {
            return numPrism * i + j;                    //jの値を上で調整して加算
        }
    }




    private float ReturnCorrectDiameter(float y){

        //手先位置の直径
        float handPosDiameter=0.2f;

        //手先一の高さ
        float maxHigh=1.5f;
        
        //傾き
        float gradient=0.0f;
        gradient=(radiusBase-handPosDiameter)/(0-maxHigh);

        float diameters=0.0f;
        if(y<maxHigh){
            diameters=gradient*y+radiusBase;
        }
        else{
            diameters=handPosDiameter;
        }
        

        return diameters;
        
    }
    //strutScaleの計算
    private float[] ReturnAllStrutScale(){
        
        //高さ求めるための関数の傾き
        float gradient=1.5f/(float)numLayer;
        float high=0.0f;
        
        float[] strutScales;
        strutScales=new float[numLayer];
        for(int i=0;i<numLayer;i++){
            //
            high=gradient*(float)(i+1);
            strutScales[i]=ReturnStrutScale(high);
        }
        return strutScales;
    }
    private float ReturnStrutScale(float high){

        float[] gradient;
        gradient=new float[9]{1.10702915f, -0.00298746f, -0.64582881f, -0.19401275f, -0.01718615f, 0.19404533f,  0.01007787f,  0.012625f,    0.00134541f};
        float intercept=1.13183542f;

        float scale=intercept;

        //high
        scale=scale+high*gradient[0];
        //layer
        scale=scale+(float)numPrism*gradient[1];

        //high^2
        scale=scale+(float)Math.Pow((double)high, 2)*gradient[2];
        //high*layer
        scale=scale+high*(float)numPrism*gradient[3];

        //layer^2
        scale=scale+(float)Math.Pow(numPrism, 2)*gradient[4];
        //high^3
        scale=scale+(float)Math.Pow((double)high, 3)*gradient[5];

        //high^2 * layer
        scale=scale+(float)Math.Pow((double)high, 2)*(float)numPrism*gradient[6];
        //high * layer^2
        scale=scale+high*(float)Math.Pow(numPrism, 2)*gradient[7];

        // layer^3
        scale=scale+(float)Math.Pow(numPrism, 3)*gradient[8];


        return scale;
    }
    //edgeloopの計算
    private float[] ReturnAllEdgeloops(){
        float[] edgeloops;
        edgeloops=new float[numLayer];
        //高さ求めるための関数の傾き
        float gradient=1.5f/(float)numLayer;
        float high=0.0f;
        
        for(int i=0;i<numLayer;i++){
            //
            high=gradient*(float)(i+1);
            edgeloops[i]=ReturnEdgeloops(high);
        }
        return edgeloops;
    }
    private float ReturnEdgeloops(float high){

        float[] gradient;;
        gradient=new float[9] {-0.04619168f, -0.00131427f, -0.0440207f, 0.00835317f, -0.00755944f, 0.04966269f,  -0.0090757f,  0.00066495f,    0.00061449f};
        float intercept=0.23052367f;

        float loop=intercept;

        //high
        loop=loop+high*gradient[0];
        //layer
        loop=loop+(float)numPrism*gradient[1];

        //high^2
        loop=loop+(float)Math.Pow((double)high, 2)*gradient[2];
        //high*layer
        loop=loop+high*(float)numPrism*gradient[3];

        //layer^2
        loop=loop+(float)Math.Pow(numPrism, 2)*gradient[4];
        //high^3
        loop=loop+(float)Math.Pow((double)high, 3)*gradient[5];

        //high^2 * layer
        loop=loop+(float)Math.Pow((double)high, 2)*(float)numPrism*gradient[6];
        //high * layer^2
        loop=loop+high*(float)Math.Pow(numPrism, 2)*gradient[7];

        // layer^3
        loop=loop+(float)Math.Pow(numPrism, 3)*gradient[8];


        return loop;
    }

    /*
    public (float scale,float high, float[] highs) simulateTensegritylength(double diffPhi   , double diffPsi , float  diffRadius){
        //柱の長さ
        float strutLength=0.0f;
        float baseStrutLength=0.0f;
        float scale=1.0f;
        float edgeloop_=0.0f;

        //テンセグリティの高さ
        float high=0.0f;
        //プサイ，φ
        double psi=0;
        double phi=0;

        //配列の設定
        var highs = new float[numLayer]; 

        while(true){
            strutLength=scale*0.2f;
            baseStrutLength=scale*0.25f;

            psi=(90- diffPsi*1)*  (float)Math.PI/180;
            high=baseStrutLength*(float)Math.Sin(psi);
            highs[0]=high;

            for(int i=2;i<=numLayer; i++){//できたら，iごとのhighを配列に格納したいな～～！！(8/22)
                phi=(90- diffPhi*i)*  Math.PI/180;
                psi=(90- diffPsi*i)*  Math.PI/180;
                edgeloop_ = (2.0f*(radiusBase - diffRadius*((float)(i-1)))*(float)Math.Sin(Math.PI/(2*numPrism)))*((float)Math.Cos((phi)/180));
                high= high + (strutLength * (float)Math.Sin(psi)) - (edgeloop_ * (float)Math.Cos(phi));
                highs[i-1]=high;
            
            } 

            if(high <= 1.55){
                break;
            } 
            scale-=0.05f;//
        } 

        return (scale,high,highs);//ここでどうにかして配列返したいな～～～！！！(8/22)
    }*/

    

    //Sceneの変更のチェックのトリがー
    private float diffRadiusLoop;
    private float diffspringForce;
    private float diffStrutScale;

    private int mode=0;

    //初期化
    public bool initializer=true;
    
    // Start is called before the first frame update        最初の初期起動時の処理
    void Start()
    {
        Time.timeScale=10.0f;

        //radiusLoopが内部に存在していた場合取得
        
        if (!initializer){
            if(PlayerPrefs.HasKey("RadiusLoop")){
            radiusLoop=PlayerPrefs.GetFloat("RadiusLoop");
            }

            if(PlayerPrefs.HasKey("SpringForce")){
                springForce =PlayerPrefs.GetFloat("SpringForce");
            }
            


            if(PlayerPrefs.HasKey("edge")){
        
                edgeparameter_layers=PlayerPrefs.GetFloat("edge");
            }
            if(PlayerPrefs.HasKey("mode")){
                mode=PlayerPrefs.GetInt("count");
            }
        }
        
        

        
        diffRadiusLoop=radiusLoop;
        diffspringForce=springForce;


        psi0=90;
        phi0=90;
        psiFin=90;
        phiFin=90;

        double diffphi = (phiFin-phi0)/(numLayer-1);
        double diffpsi = (psiFin-psi0)/numLayer;

        float lastRadiusloop = radiusBase*edgeDampoer;
        float diffRadius = (radiusBase-lastRadiusloop)/((float)(numLayer-1));      //上と下の半径の差

        edgeLoop=new float[numLayer];
        StrutScale=new float[numLayer];

        int bases = numLayer/5;

        //高さ計算
        //var result=simulateTensegritylength(diffphi,diffpsi,diffRadius);
        //StrutScale=result.scale;
        //predict=result.high;
        //var sumHigh = new float[numLayer];
        //sumHigh=result.highs; 
        //Debug.Log(result);

    
        
        // Allocate variables                           //(オブジェクトの入れものとなる配列生成)
        baseblocks = new GameObject[numPrism];          //土台の原点の生成(地面の四角のやつ)
        struts = new GameObject[numPrism*numLayer];     //柱の生成
        structsEnd1 = new Transform[numPrism*numLayer];
        structsEnd2 = new Transform[numPrism*numLayer];
        loops = new GameObject[(numLayer+1)];           //柱の点の丸いやつ
        stripes = new GameObject[numPrism*2];           //筋肉(紐)の数(白い紐)

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
            if (useConstant==true){

            
                if(numLayer==10 && numPrism==8){//
                    StrutScale=new float[10]{0.7741794f,0.7375128f,0.805346f,0.7799709f,0.7509292f,0.7203457f,0.7009292f,0.6540951f,0.6223454f,0.6304705f};
                    edgeLoop=new float[10]{0.06268743f,0.06676555f,0.06041659f,0.05430724f,0.05361452f,0.05094265f,0.04656245f,0.04400515f,0.04369787f,0.0630364f};     //640
            
                }
                if(numLayer==10 && numPrism==7){//
                    StrutScale=new float[10]{1.004277f,0.9194543f,0.8781046f,0.8150301f,0.8156192f,0.7553421f,0.71342f,0.6632004f,0.6456949f,0.6138333f};
                    edgeLoop=new float[10]{0.08695613f,0.0828373f,0.07941385f,0.07400092f,0.07087807f,0.06425336f,0.06047757f,0.05544934f,0.04867428f,0.09390821f};     //640
            
                }
                if(numLayer==10 && numPrism==6){//
                    StrutScale=new float[10]{1.027173f,0.9488513f,0.9043089f,0.8632551f,0.8158698f,0.7711644f,0.7323623f,0.6897659f,0.6515551f,0.6174093f};
                    edgeLoop=new float[10]{0.1031376f,0.09988528f,0.09375463f,0.08812632f,0.08236708f,0.07627424f,0.07092295f,0.06517997f,0.05856487f,0.09897218f};     //640
            
                }
                if(numLayer==10 && numPrism==5){//
                    StrutScale=new float[10]{0.9954582f,0.9390308f,0.9357504f,0.9445467f,0.9125582f,0.9185466f,0.8610252f,0.8345063f,0.7915034f,0.7884449f};
                    edgeLoop=new float[10]{0.1124301f,0.1152653f,0.1178389f,0.1132898f,0.1109571f,0.1123632f,0.09783024f,0.0997474f,0.09700358f,0.1247879f};     //880
            
                }
                else if(numLayer==8 && numPrism==5){//
                //StrutScale=new float[8]{0.8564691f,0.903886f,0.8937612f,0.879386f,0.7824689f,0.773761f,0.7320107f,0.7030106f};
                //edgeLoop=new float[8]{0.09180925f,0.08973639f,0.08356636f,0.07646576f,0.07029568f,0.06472287f,0.05919171f,0.09743829f};     //1120
                    StrutScale=new float[8]{1.089899f,1.016552f,0.9621532f,0.9116253f,0.8568946f,0.8094475f,0.7564716f,0.7444379f};              //640
                    edgeLoop=new float[8]{0.1257394f,0.118149f,0.1103842f,0.1007877f,0.09299219f,0.08393376f,0.07367938f,0.1448224f};     //640
                }   
                else if(numLayer==8 && numPrism==8){//
                    StrutScale=new float[8]{1.01541f,0.9374977f,0.891705f,0.852612f,0.8004768f,0.7549734f,0.7101514f,0.6713954f};
                    edgeLoop=new float[8]{0.07377467f,0.07200986f,0.0667648f,0.06203499f,0.05646168f,0.05149865f,0.04500595f,0.07584359f};     //640
                }   

                else if(numLayer==4 && numPrism==3){//
                    StrutScale=new float[4]{1.431883f,1.448337f,1.37506f,1.268907f};
                    edgeLoop=new float[4]{0.2097509f,0.185938f,0.1771897f,0.2095114f};     //800
                }   
                else if(numLayer==4 && numPrism==4){//
                    StrutScale=new float[4]{1.389812f,1.352098f,1.294913f,1.171735f};
                    edgeLoop=new float[4]{0.1567681f,0.1381599f,0.121954f,0.1414613f};     //640
                }
                else if(numLayer==4 && numPrism==5){//
                    StrutScale=new float[4]{1.284886f,1.084796f,1.088967f,0.9954147f};
                    edgeLoop=new float[4]{0.07766558f,0.09534243f,0.06815712f,0.09635011f};     //720
                }
                else if(numLayer==4 && numPrism==6){//
                    StrutScale=new float[4]{1.317688f,1.286952f,1.223615f,1.121616f};
                    edgeLoop=new float[4]{0.1028999f,0.09467847f,0.08020752f,0.09932601f};     //640
                }
                else if(numLayer==4 && numPrism==8){//none
                    StrutScale=new float[4]{1.149631f,0.9640459f,0.9619905f,0.83283f};
                    edgeLoop=new float[4]{0.002082939f,-0.128569f,0.00714466f,0.05683319f};     //1520
                }
                else if(numLayer==5 && numPrism==4){//
                    StrutScale=new float[5]{1.258871f,1.21481f,1.166965f,1.167346f,0.9817054f};
                    edgeLoop=new float[5]{0.1546911f,0.1415395f,0.1383539f,0.1161737f,0.1410651f};     //640
                }
                else if(numLayer==5 && numPrism==7){//
                    StrutScale=new float[5]{1.175833f,1.122695f,1.064568f,1.002691f,0.939831f};
                    edgeLoop=new float[5]{0.08672506f,0.08115824f,0.07188646f,0.06030932f,0.09528822f};     //640
                }
                else if(numLayer==6 && numPrism==4){
                    StrutScale=new float[6]{1.174267f,1.138367f,1.073669f,1.009512f,0.952307f,0.8609945f};
                    edgeLoop=new float[6]{0.1522418f,0.144127f,0.1302245f,0.1157298f,0.1017524f,0.1415865f};     //640
                }
                else if(numLayer==6 && numPrism==5){//
                    StrutScale=new float[6]{1.15767f,1.097207f,1.037237f,0.976959f,0.9139902f,0.8725966f};
                    edgeLoop=new float[6]{0.1250576f,0.1157846f,0.1049821f,0.09368186f,0.07977708f,0.1449054f};     //640
                }
                else if(numLayer==6 && numPrism==6){//
                    StrutScale=new float[6]{1.127176f,1.062826f,1.005244f,0.9504106f,0.8919847f,0.8256059f};
                    edgeLoop=new float[6]{0.1021722f,0.09579851f,0.08671429f,0.07753801f,0.06732567f,0.09927308f};     //640
                }
                else if(numLayer==6 && numPrism==8){//none 
                    StrutScale=new float[6]{0.8890916f,0.8968414f,0.8968414f,0.7916886f,0.7295395f,0.721164f};
                    edgeLoop=new float[6]{0.03823249f,0.04800971f,0.04851066f,0.04167225f,0.03886984f,0.05012858f};     //1040
                }
                else if(numLayer==7 && numPrism==4){//
                    StrutScale=new float[7]{1.00664f,1.033036f,1.058159f,1.064581f,1.018653f,1.000438f,1.009503f};
                    edgeLoop=new float[7]{0.1401186f,0.137489f,0.121433f,0.1284183f,0.1219166f,0.1274466f,0.1894173f};     //800
                }
                else if(numLayer==7 && numPrism==5){//
                    StrutScale=new float[7]{1.084986f,1.027027f,0.9862122f,0.9437142f,0.8776177f,0.8190169f,0.8020871f};
                    edgeLoop=new float[7]{0.1170012f,0.1141681f,0.1071079f,0.09556685f,0.08723921f,0.07619226f,0.1442267f};     //640
                }
                else if(numLayer==7 && numPrism==6){//
                    StrutScale=new float[7]{1.081339f,1.021421f,0.9597515f,0.9069321f,0.8520491f,0.8037038f,0.7468492f};
                    edgeLoop=new float[7]{0.1016314f,0.09649281f,0.0883932f,0.08012963f,0.07254038f,0.06358072f,0.09843413f};     //800
                }
                else if(numLayer==7 && numPrism==7){//
                    StrutScale=new float[7]{1.061331f,0.9894107f,0.9334263f,0.8938653f,0.8429952f,0.7789527f,0.7505084f};
                    edgeLoop=new float[7]{0.0857067f,0.0826979f,0.0755123f,0.06978621f,0.06210569f,0.05418411f,0.09542076f};     //640
                }
                else if(numLayer==7 && numPrism==8){//
                    StrutScale=new float[7]{1.044672f,0.9696441f,0.9240914f,0.8684586f,0.8200766f,0.7746933f,0.7273475f};
                    edgeLoop=new float[7]{0.07340466f,0.07200859f,0.06498464f,0.05982734f,0.0532186f,0.04739253f,0.07478108f};     //640
                }
                else if(numLayer==8 && numPrism==4){//
                    StrutScale=new float[8]{1.111916f,1.05748f,1.092146f,0.9202463f,0.8956519f,0.8381414f,0.7887803f,0.7243479f};
                    edgeLoop=new float[8]{0.1539257f,0.1557249f,0.1336584f,0.1278006f,0.1146122f,0.1046207f,0.0936265f,0.1416934f};     //640
                }
                else if(numLayer==8 && numPrism==6){//
                    StrutScale=new float[8]{1.016868f,0.9480168f,0.8878636f,0.9155759f,0.8251612f,0.7715869f,0.7201943f,0.7089073f};
                    edgeLoop=new float[8]{0.09447099f,0.09396244f,0.08926377f,0.08497817f,0.07454205f,0.06858622f,0.06306095f,0.0926666f};     //640
                }
                



            }
            else if(useFunc){
                StrutScale=ReturnAllStrutScale();
                edgeLoop = new float[numLayer];
                edgeLoop=ReturnAllEdgeloops();
            }

            else{
                for(int i=0;i<numLayer;i++){

                    StrutScale[i]=1.0f;
                    Debug.Log(edgeLoop[i]);
                }
            }

 
            
            //StrutScale=new float[13]{0.5066695f,0.4845017f,0.4922935f,0.4921669f,0.5023773f,0.5407119f,0.5237533f,0.5343785f,0.5101283f,0.5261283f,0.5105029f,0.5381283f,0.5263783f};

            
            //edgeLoop=new float[13]{0.05042118f,0.05176077f,0.0528837f,0.03907325f,0.03636284f,0.03176075f,0.028792f,0.02953992f,0.02667118f,0.02521077f,0.02398368f,0.02277328f,0.03694427f};



            //パラメータの引継ぎ
            /*
            for(int i=0;i<numLayer;i++){
                PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
                PlayerPrefs.Save();
            }
            */
            
           

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

            
        

        else{

            for (int i=0;i<numLayer;i++)
            {
                StrutScale[i]=PlayerPrefs.GetFloat("StrutScale"+(i).ToString());
                
                edgeLoop[i]=PlayerPrefs.GetFloat("edgeLoop"+(i).ToString());
                //Debug.Log("edgeLoop"+(i).ToString());
                //Debug.Log(edgeLoop[i]);
                
                //4本5層の値
                //edgeLoop= new float[5] {0.12415123082906993f,0.11031530609675386f,0.10214850457080119f,0.09057431266995283f,0.12024559516481466f};
                //edgeLoop=new float[10]{0.05f, 0.04f, 0.03f, 0.02857142857142857f, 0.027142857142857142f, 0.025714285714285714f, 0.024285714285714285f, 0.022857142857142857f, 0.02142857142857143f, 0.02f};
            
                /*
                if(mode==0){
                    edgeLoop[i-1]=   (2.0f*(radiusBase - diffRadius*((float)(i)))*(float)Math.Sin(Math.PI/(2*numPrism)))*((float)Math.Cos((diffphi*i*Math.PI)/180));
                }
                else if(mode==1){
                    radius=radiusBase-(((1-edgeDampoer)*radiusBase)/predict * sumHigh[i-1]);
                
                    edgeLoop[i-1]=   2.0f * radius *(float)Math.Sin(Math.PI/(2*numPrism))*((float)Math.Cos((diffphi*i*Math.PI)/180));
                }
           
                if(mode==0){
                    radius=radiusBase-(((1-edgeDampoer)*radiusBase)/predict * sumHigh[i-1]);
                
                    edgeLoop[i-1]=   2.0f * radius *(float)Math.Sin(Math.PI/(2*numPrism))/((float)Math.Cos((diffphi*i*Math.PI)/180));
                }
                else if(mode==1){
                
                    edgeLoop[i-1]=   (2.0f*(radiusBase - diffRadius*((float)(i)))*(float)Math.Sin(Math.PI/(2*numPrism)))*((float)Math.Cos((diffphi*i*Math.PI)/180));
                }
                else{
                    edgeLoop[i-1]=   (2.0f*(radiusBase - diffRadius*((float)(i)))*(float)Math.Sin(Math.PI/(2*numPrism)))*((float)Math.Cos((diffphi*i*Math.PI)/180));
                }
                //Debug.Log(Math.Cos(phi*i));
                //edgeLoop[i-1] =    StrutScale *    (2.0f *(radiusBase - diffRadius*((float)(i+1)))*(float)Math.Sin(Math.PI/(2*numPrism)));
            
                //radius=(0.6f*radiusBase/predict)*
                //edgeLoop[i-1]=   (2.0f*;
                //2rsin(θ)/(縦側の角度φのcos)
                */
            } 

        }
        //float masses= (0.9f)/(float)(numLayer*numPrism);
        
        //edgeLoop= new float[10] {0.05733649f,0.0482649f,0.0458912f,0.04588119f,0.04587119f,0.02872325f,0.02871325f,0.02873325f,0.02870325f,0.02871325f};
        //slackで送ったやつ//edgeLoop= new float[10] {0.05133649f,0.0482649f,0.0458912f,0.04588119f,0.04587119f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; //base0.65 scale 0.6 
        //edgeLoop= new float[10] {0.05133649f, 0.0495571124f,0.0477777348f, 0.0459983572f,0.0442189796f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; //base0.65 scale 0.6 
        //edgeLoop= new float[10] {0.04833649f, 0.0471571124f, 0.0459777348f, 0.0447983572f,0.0436189796f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; //base0.65 scale 0.6 
        //今のところいいやつ//edgeLoop= new float[10] {0.04833649f, 0.0471571124f, 0.0459777348f, 0.0447983572f,0.0436189796f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; 
        // Place the bottom layer's parts and connect ball joints　　//一番下の層の制作
        
        for (int i = 0; i < numPrism ; i++)
        {
            var step = 2*Math.PI/numPrism;
            if(earthConnected==true){
                // Instantiate and place prefabs
                baseblocks[i] = Instantiate(basePlate, this.transform);                             //basePlateプレハブからobjectを生成し，先ほどの配列に格納                  
                baseblocks[i].transform.position = this.transform.position + new Vector3(           //baseblocksの座標の配置 
                    radiusBase*(float)Math.Cos(step*i),                                             //x座標はradiusBase(半径)×cos((2π/ストラットの数)×i)極座標→直交座標への変換プロセス
                    0,                                                                              //y=0(土台のため)
                    radiusBase*(float)Math.Sin(step*i));                                            //z=radiusBase(半径)×sin((2π/ストラットの数)×i)極座標→直交座標への変換プロセス
                baseblocks[i].name = $"Base{i}";                                                    //名前の決定(object管理のため)
            
            }
            struts[i] = Instantiate(baseStrut, this.transform);                                 //ストラクト(支柱)の制作
            struts[i].transform.localScale = new Vector3(0.02f,(float)(StrutScale[0]*0.20f),0.02f); //デフォルトは0.25        
            struts[i].transform.position = this.transform.position + new Vector3(               //ストラクトの座標と生成(処理は上と同じ)
                radiusBase*(float)Math.Cos(step*i), 
                0.215f*StrutScale[0]
                ,                                                                         //土台の高さがある分少し高い？
                radiusBase*(float)Math.Sin(step*i)
                );
            struts[i].name = $"Strut{i}";
            // Connect joints                                                                   
            ConfigurableJoint basejoint = struts[i].GetComponent<ConfigurableJoint>();          //
            //

            //地面から外したらこうなる
            if(earthConnected==false){
                basejoint.xMotion=ConfigurableJointMotion.Free;
                basejoint.yMotion=ConfigurableJointMotion.Free;
                basejoint.zMotion=ConfigurableJointMotion.Free;

                basejoint.angularXMotion=ConfigurableJointMotion.Free;
                basejoint.angularYMotion=ConfigurableJointMotion.Free;
                basejoint.angularZMotion=ConfigurableJointMotion.Free;
            }
            else{
                basejoint.connectedBody = baseblocks[i].GetComponent<Rigidbody>();                  //接続処理
            }

            var baseRigidBody=struts[i].GetComponent<Rigidbody>();
            //baseRigidBody.mass=0.1f*StrutScale[0];
            //baseRigidBody.mass=masses;

        }

        //Strutのy座標
        float strutPositionY=0.0f;
        float strutRadius=0.0f;

        // Place the rest layers' struts                                                        //柱の生成
        for (int i = 1; i < numLayer ; i++)                                                     //
        {
            var step = 2*Math.PI/numPrism;                                                      //r-θのθ部分の決定に使う 
            //var twist = Math.PI/numPrism;   
            var twist = Math.PI/numPrism*(1/3);                                                 //i%2により，互い違いになるように柱を並べるための変数．これにより，いい感じにずれる．
            // Instantiate and place the prefab

            strutPositionY=(0.215f*StrutScale[0])+0.5f*(0.2f*StrutScale[0]);
            for(int j=1;j<=i;j++){
                 strutPositionY+=0.5f*StrutScale[j];
            }
            strutRadius=ReturnCorrectDiameter(strutPositionY);
            strutRadius=strutRadius/2.0f;
            //Debug.Log(strutRadius);

            for (int j = 0; j < numPrism ; j++)
            {
                

                struts[index(i, j)] = Instantiate(middleStrut, this.transform);                 //生成i,jの2次元配列にすることで，層と層の何個目かわかる
                

                struts[index(i, j)].transform.localScale = new Vector3(0.02f,(float)(StrutScale[i]*0.20f),0.02f); //デフォルトは0.2 StrutScale*
                struts[index(i, j)].transform.position = this.transform.position + new Vector3( //座標
                    strutRadius*(float)Math.Cos(step*j+twist*(i%2)),                        
                    strutPositionY,                                                      //y 積みあがる高さ分加算   データ収集後こっちの式でやってみる
                    strutRadius*(float)Math.Sin(step*j+twist*(i%2))                         //z 
                );  
                
                

                
                structsEnd1[index(i, j)] = struts[index(i, j)].transform.GetChild(0);
                structsEnd2[index(i, j)] = struts[index(i, j)].transform.GetChild(1);
                structsEnd1[index(i, j)].transform.localScale = new Vector3(1.5f,0.15f,1.5f);
                structsEnd2[index(i, j)].transform.localScale = new Vector3(1.5f,0.15f,1.5f);
                
                                                                                        //rsinΘ,rcosΘで点の位置(ストラットの先端の位置が決まるが，ばねの最大の長さ，stripeshapeを変更しないと収縮する)
                struts[index(i, j)].name = $"Strut{index(i,j)}";                                //index関数から返された値の名前を付ける

                //var struitRigidBody=struts[i].GetComponent<Rigidbody>();
                //struitRigidBody.mass=0.1f*StrutScale[i];
                //struitRigidBody.mass=masses;
                
            }
        }
        
        var isMatched = edgeLoop.GetLength(0) == numLayer;
        //Debug.Log(isMatched);
        // Connect & configure spring joints                                                    //接続する処理
        for (int i = 0; i < numLayer ; i++)                                                     //積層する数だけ周回
        {
            // prepare the variable to express the twisting direction in the i-th layer         //
            var twist_dir = -1;
            if (i%2 == 0) {
                twist_dir = 1;
            }

            // connect springs                                                                  //ばねの接続は柱の数だけやる
            for (int j = 0; j < numPrism ; j++)
            {
                SpringJoint[] spring = struts[index(i, j)].GetComponents<SpringJoint>();        //物理演算のSpringJointを使用　柱同士の接続　(ばねの様にオブジェクト同士を接続するもの)
                
                // Each strut has 4 springs
                // prepare target indice
                // *** Layer-dependent exceptions will be addressed later ***
                
                var target1 = index(i+2, j+twist_dir);                                          //接続先のターゲットを決定している
                var target2 = index(i+1, j);                                                    //接続先の配列呼び出しのための数字の決定
                var target3 = index(i, j+twist_dir);
                var target4 = index(i-1, j-twist_dir);



                //connected anochor のvectorにより，Asset内のプレハブのローカル位置を指定して結び付けしている
                // Connect springs while dealing with layer-dependent exceptions
                // #1
                if (i == numLayer-1) {
                    
                    target1 = index(i, j+twist_dir);
                    //Debug.Log(spring[0].damper);
                    //spring[0].spring = springForce;
                    //spring[0].spring = springForce+(SpringDmaper*(numLayer-i)*(numLayer-i));                                                    //ばねの強さの設定
                    spring[0].connectedAnchor = new Vector3(0, 1, 0);                           //親オブジェクトとのローカル位置
                } else if (i == numLayer-2) {
                    target1 = index(i+1, j+twist_dir);
                    spring[0].connectedAnchor = new Vector3(0, 1, 0);
                } else {
                    spring[0].connectedAnchor = new Vector3(0, -1, 0);
                }
                spring[0].anchor = new Vector3(0, 1, 0);                                        //オブジェクトのローカル空間の位置
                spring[0].connectedBody = struts[target1].GetComponent<Rigidbody>();            //接続されるオブジェクトの設定
                spring[0].enableCollision=springCollision;
                spring[0].spring=springForce;
                
                // #2
                if (i == numLayer-1) {
                    target2 = index(i, j+twist_dir);
                    spring[1].connectedAnchor = new Vector3(0, 1, 0);
                } else {
                    spring[1].connectedAnchor = new Vector3(0, -1, 0);
                }
                spring[1].anchor = new Vector3(0, 1, 0);
                spring[1].connectedBody = struts[target2].GetComponent<Rigidbody>();
                if (isMatched) {
                    spring[1].tolerance = edgeLoop[i]*0.001f; 
                                     //復元力が働く距離と働かない距離の誤差の設定
                    spring[1].minDistance = edgeLoop[i]*0.5f;
                    spring[1].maxDistance = edgeLoop[i];  
                    //spring[1].spring=Mathf.Infinity;
                    //spring[1].damper=400000;


                }
                spring[1].enableCollision=springCollision;
                
                // #3
                spring[2].connectedBody = struts[target3].GetComponent<Rigidbody>();
                spring[2].connectedAnchor = new Vector3(0, -1, 0);
                spring[2].anchor = new Vector3(0, 1, 0);
                spring[2].enableCollision=springCollision;
                spring[2].spring=springForce;


                // #4
                if (i == 0 && i != numLayer-1) {
                    target4 = index(i+1, j-2*twist_dir);
                    spring[3].connectedAnchor = new Vector3(0, -1, 0);
                } else if (i == 0 && i == numLayer-1) {
                    target4 = index(i, j-twist_dir);
                    spring[3].connectedAnchor = new Vector3(0, -1, 0);
                    //spring[3].spring = springForce;
                    //spring[3].spring = springForce+(SpringDmaper*(numLayer-i)*(numLayer-i));
                } else {
                    spring[3].connectedAnchor = new Vector3(0, 1, 0);
                    if (isMatched) {
                        spring[3].tolerance = edgeLoop[i-1]*0.001f;
                        spring[3].minDistance = edgeLoop[i-1]*0.5f;
                        spring[3].maxDistance = edgeLoop[i-1];  
                        //spring[3].spring=Mathf.Infinity;
                        //spring[1].damper=400000;
                        

                    }
                }

                
                spring[3].connectedBody = struts[target4].GetComponent<Rigidbody>();                //
                spring[3].anchor = new Vector3(0, -1, 0);
                spring[3].enableCollision=springCollision;
            }
        }

    


        ///*
        // Add loop renderers  #横の紐の描写?
        for (int i = 0; i <= numLayer ; i++) 
        {
            
            loops[i] = Instantiate(loopLine, this.transform);
            loops[i].name = $"Loop Renderer{i}";   
            //点と点をつなげるオブジェクト                     
            LineRenderer line = loops[i].GetComponent<LineRenderer>();      //紐の
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.widthMultiplier = 0.005f;                                  //紐の太さ  //当たり判定等を入れれば
            line.loop = true;
            line.numCapVertices = 1;
            line.numCornerVertices = 1;

            

            if (i == 0 || i == numLayer) 
            {
                line.positionCount = numPrism;
            } 
            else {
                line.positionCount = numPrism*2;
            }

            loops[i].AddComponent<BoxCollider>();



            
        }
        //*/

        // Add stripe renderers  縦の紐の描写
        for (int i = 0; i < numPrism*2 ; i++) 
        {
            stripes[i] = Instantiate(vertcalLine, this.transform);
            stripes[i].name = $"Stripes Renderer{i}";
            LineRenderer line = stripes[i].GetComponent<LineRenderer>();
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.widthMultiplier = 0.005f;
            line.loop = false;
            line.numCapVertices = 1;
            line.numCornerVertices = 1;
            line.positionCount = numLayer+1;
        }
        
        if (isPausedStart) {
            UnityEditor.EditorApplication.isPaused = true;
        }

        if (isNeverSleep) {
            for (int i = 0; i < numLayer; i++) {
                for (int j = 0; j < numPrism; j++) {
                    Rigidbody rb = struts[index(i, j)].GetComponent<Rigidbody>();
                    rb.sleepThreshold = -1;
                }
            }
        }

        for(int i=0;i<numLayer;i++){
            PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
            //Debug.Log("edgeLoop"+(i).ToString());
            //Debug.Log(edgeLoop[i]);
            PlayerPrefs.Save();
            PlayerPrefs.SetFloat("StrutScale"+(i).ToString(),StrutScale[i]);
            PlayerPrefs.Save();
        }

        PlayerPrefs.SetFloat("RadiusLoop",radiusLoop);
        PlayerPrefs.Save();
        PlayerPrefs.SetFloat("SpringForce",springForce);
        PlayerPrefs.Save();
        


        isReady = true;
    }

    // Update is called once per frame
    void Update()
    {      

        
        
        

        
        // Update loop renderers
        for (int i = 0; i <= numLayer ; i++) 
        {
            LineRenderer line = loops[i].GetComponent<LineRenderer>();
            if (i == 0) {
                for (int j = 0; j < numPrism ; j++) {
                    line.SetPosition(j, struts[j].transform.GetChild(1).transform.position);
                }
            } else if (i == numLayer) {
                for (int j = 0; j < numPrism ; j++) {
                    line.SetPosition(j, struts[index(numLayer-1,j)].transform.GetChild(0).transform.position);
                }
            } else {
                if(i%2 == 1){
                    for (int j = 0; j < numPrism ; j++) {
                        line.SetPosition(2*j, struts[index(i-1,j)].transform.GetChild(0).transform.position);
                        line.SetPosition(2*j+1, struts[index(i,j)].transform.GetChild(1).transform.position);
                    }
                } else {
                    for (int j = 0; j < numPrism ; j++) {
                        line.SetPosition(2*j, struts[index(i,j)].transform.GetChild(1).transform.position);
                        line.SetPosition(2*j+1, struts[index(i-1,j)].transform.GetChild(0).transform.position);
                    }
                }
            }
            if(exportBotton){
                //メッシュを焼いてFBX化できるようにするためのオブジェクト
                // **新しい Mesh を作成**
                Mesh lineMesh = new Mesh();
                line.BakeMesh(lineMesh, true);

                // メッシュ用の GameObject を作成
                GameObject meshObject = new GameObject("LineMesh" + i.ToString());
                meshObject.AddComponent<MeshFilter>().mesh = lineMesh;
                meshObject.AddComponent<MeshRenderer>().material = line.material;
                meshObject.transform.parent = this.gameObject.transform;

                
            }
            
        }

        // Update stripe renderers
        for (int i = 0; i < numPrism ; i++) 
        {
            LineRenderer line1 = stripes[2*i].GetComponent<LineRenderer>();
            LineRenderer line2 = stripes[2*i+1].GetComponent<LineRenderer>();
            line1.SetPosition(0, struts[index(0,i)].transform.GetChild(1).transform.position);
            line2.SetPosition(0, struts[index(0,i+1)].transform.GetChild(1).transform.position);
            for (int j = 1; j <= numLayer ; j++) 
            {
                if ((j/2)*2 < numLayer) {
                    line1.SetPosition(j, struts[index((j/2)*2, i-j%2)].transform.GetChild((j+1)%2).transform.position);
                } else {
                    line1.SetPosition(j, struts[index(numLayer-1, i+1)].transform.GetChild(0).transform.position);
                }
                if (((j-1)/2)*2+1 < numLayer) {
                    line2.SetPosition(j, struts[index(((j-1)/2)*2+1, i-j%2)].transform.GetChild(j%2).transform.position);
                } else {
                    line2.SetPosition(j, struts[index(numLayer-1, i-1)].transform.GetChild(0).transform.position);
                }
            }
            if(exportBotton){
                //メッシュを焼いてFBX化できるようにするためのオブジェクト
                // **新しい Mesh を作成**
                Mesh lineMesh = new Mesh();
                line1.BakeMesh(lineMesh, true);

                // メッシュ用の GameObject を作成
                GameObject meshObject = new GameObject("LineMesh" + i.ToString());
                meshObject.AddComponent<MeshFilter>().mesh = lineMesh;
                meshObject.AddComponent<MeshRenderer>().material = line1.material;
                meshObject.transform.parent = this.gameObject.transform;

                
            }
            if(exportBotton){
                //メッシュを焼いてFBX化できるようにするためのオブジェクト
                // **新しい Mesh を作成**
                Mesh lineMesh = new Mesh();
                line2.BakeMesh(lineMesh, true);

                // メッシュ用の GameObject を作成
                GameObject meshObject = new GameObject("LineMesh" + i.ToString());
                meshObject.AddComponent<MeshFilter>().mesh = lineMesh;
                meshObject.AddComponent<MeshRenderer>().material = line2.material;
                meshObject.transform.parent = this.gameObject.transform;

                
            }
        }

        if(exportBotton){
            exportBotton=false;
        }

        

    }

}
