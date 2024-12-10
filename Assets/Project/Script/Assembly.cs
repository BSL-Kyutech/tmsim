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
            return numPrism * i + j;                    //numPrismが0でないとうまくいかない？どこかでnumPrismは初期に0？どこかで受け取っている？
        }
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
        Time.timeScale=100.0f;

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

        edgeLoop = new float[numLayer];
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
        

        float first_edge=0.06676555f;
        float last_edge=0.04369787f;
        float onlyedge=0.06268743f;

        if(initializer){
            
            for (int i=1;i<=numLayer;i++){
                //edgeLoop[i-1]= (2.0f*(radiusBase - diffRadius*((float)(i)))*(float)Math.Sin(Math.PI/(2*numPrism)))*0.65f;
                
                StrutScale[i-1]=0.8f-(0.8f-0.60f)/numLayer * (i-1);
                //StrutScale[i-1]=0.85f-(0.8W5f-0.7f)/numLayer * (i-1);
                if(i==numLayer){
                    edgeLoop[i-1]=onlyedge;
                }
                else if(i==0){
                    edgeLoop[i-1]=onlyedge*0.9f;
                }
                else{
                    edgeLoop[i-1]=(first_edge-(first_edge-last_edge)/(numLayer-2) * (i-2))*0.65f;
                }
            }

            //StrutScale=new float[10]{0.7112515f,0.7213767f,0.7011681f,0.6990014f,0.6775014f,0.6721681f,0.6682513f,0.6419179f,0.6465013f,0.6452513f};
            
            //edgeLoop=new float[10]{0.05607285f,0.05599472f,0.05252077f,0.05004682f,0.05190619f,0.04651557f,0.04581245f,0.03875516f,0.03686455f,0.0277031f};



            //パラメータの引継ぎ
            for(int i=0;i<numLayer;i++){
                PlayerPrefs.SetFloat("edgeLoop"+(i).ToString(),edgeLoop[i]);
                PlayerPrefs.Save();
            }
            
           

            //edgeLoop=new float[10]{0.065649f, 0.05787f, 0.05438851111111f, 0.05100837766666666f, 0.04738228702222222f, 0.043882287022222224f, 0.039228702222222f, 0.03882287022222223f, 0.03838228702222223f, 0.054f};
            //edgeLoop=new float[10]{0.065649f, 0.06289442f, 0.0731718f, 0.04244859f, 0.04472593f, 0.0510033f, 0.05128067f, 0.04755802f, 0.04683538f, 0.03011275f};
            //edgeLoop=new float[10]{0.05314891f, 0.04836987f, 0.04488838f, 0.04350825f, 0.03888217f, 0.03488217f, 0.02922862f, 0.03382276f, 0.03138219f, 0.04899989f};
            initializer=false;
            PlayerPrefs.SetInt("compareFlag",0);//compareflagの初期化は後でコメントアウト外さないといけない
            PlayerPrefs.Save();
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

            for (int i=1;i<=numLayer;i++)
            {
            
                edgeLoop[i-1]=(2.0f*(radiusBase - diffRadius*((float)(i)))*(float)Math.Sin(Math.PI/(2*numPrism)))*((float)Math.Cos((diffphi*i*Math.PI)/180));
                if(PlayerPrefs.HasKey("edgeLoop"+(i-1).ToString())){
                    edgeLoop[i-1]=PlayerPrefs.GetFloat("edgeLoop"+(i-1).ToString());
                }
                if(PlayerPrefs.HasKey("StrutScale"+(i-1).ToString())){
                    StrutScale[i-1]=PlayerPrefs.GetFloat("StrutScale"+(i-1).ToString());
                }
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
        
        
        //edgeLoop= new float[10] {0.05733649f,0.0482649f,0.0458912f,0.04588119f,0.04587119f,0.02872325f,0.02871325f,0.02873325f,0.02870325f,0.02871325f};
        //slackで送ったやつ//edgeLoop= new float[10] {0.05133649f,0.0482649f,0.0458912f,0.04588119f,0.04587119f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; //base0.65 scale 0.6 
        //edgeLoop= new float[10] {0.05133649f, 0.0495571124f,0.0477777348f, 0.0459983572f,0.0442189796f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; //base0.65 scale 0.6 
        //edgeLoop= new float[10] {0.04833649f, 0.0471571124f, 0.0459777348f, 0.0447983572f,0.0436189796f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; //base0.65 scale 0.6 
        //今のところいいやつ//edgeLoop= new float[10] {0.04833649f, 0.0471571124f, 0.0459777348f, 0.0447983572f,0.0436189796f,0.042439602f,0.039008014f,0.035576425999999994f,0.032144837999999995f,0.02871325f}; 
        // Place the bottom layer's parts and connect ball joints　　//一番下の層の制作
        for (int i = 0; i < numPrism ; i++)
        {
            var step = 2*Math.PI/numPrism;
            // Instantiate and place prefabs
            baseblocks[i] = Instantiate(basePlate, this.transform);                             //basePlateプレハブからobjectを生成し，先ほどの配列に格納                  
            baseblocks[i].transform.position = this.transform.position + new Vector3(           //baseblocksの座標の配置 
                radiusBase*(float)Math.Cos(step*i),                                             //x座標はradiusBase(半径)×cos((2π/ストラットの数)×i)極座標→直交座標への変換プロセス
                0,                                                                              //y=0(土台のため)
                radiusBase*(float)Math.Sin(step*i));                                            //z=radiusBase(半径)×sin((2π/ストラットの数)×i)極座標→直交座標への変換プロセス
            baseblocks[i].name = $"Base{i}";                                                    //名前の決定(object管理のため)
            struts[i] = Instantiate(baseStrut, this.transform);                                 //ストラクト(支柱)の制作
            struts[i].transform.localScale = new Vector3(0.02f,(float)(StrutScale[0]*0.20f),0.02f); //デフォルトは0.25        
            struts[i].transform.position = this.transform.position + new Vector3(               //ストラクトの座標と生成(処理は上と同じ)
                radiusBase*(float)Math.Cos(step*i), 
                0.215f*StrutScale[0]
                ,                                                                         //土台の高さがある分少し高い？
                radiusBase*(float)Math.Sin(step*i)
                );
            struts[i].name = $"Strut{i}";
            // Connect joints                                                                   //接続処理
            ConfigurableJoint basejoint = struts[i].GetComponent<ConfigurableJoint>();          //
            basejoint.connectedBody = baseblocks[i].GetComponent<Rigidbody>();                  //
        }

        // Place the rest layers' struts                                                        //柱の生成
        for (int i = 1; i < numLayer ; i++)                                                     //
        {
            var step = 2*Math.PI/numPrism;                                                      //r-θのθ部分の決定に使う 
            //var twist = Math.PI/numPrism;   
            var twist = Math.PI/numPrism*(1/3);                                                 //ねじれ？
            // Instantiate and place the prefab
            for (int j = 0; j < numPrism ; j++)
            {

                struts[index(i, j)] = Instantiate(middleStrut, this.transform);                 //生成i,jの2次元配列にすることで，層と層の何個目かわかる
                

                struts[index(i, j)].transform.localScale = new Vector3(0.02f,(float)(StrutScale[i]*0.20f),0.02f); //デフォルトは0.2 StrutScale*
                struts[index(i, j)].transform.position = this.transform.position + new Vector3( //座標
                    stripeshape*(radiusBase/1.5f)*(float)Math.Cos(step*j+twist*(i%2)),                        
                    (0.215f*StrutScale[0])+(float)(i-1)*(0.3f*StrutScale[i]),                                                      //y 積みあがる高さ分加算   データ収集後こっちの式でやってみる
                    stripeshape*(radiusBase/1.5f)*(float)Math.Sin(step*j+twist*(i%2))                         //z 
                );  
                
                

                
                structsEnd1[index(i, j)] = struts[index(i, j)].transform.GetChild(0);
                structsEnd2[index(i, j)] = struts[index(i, j)].transform.GetChild(1);
                structsEnd1[index(i, j)].transform.localScale = new Vector3(1.5f,0.15f,1.5f);
                structsEnd2[index(i, j)].transform.localScale = new Vector3(1.5f,0.15f,1.5f);
                
                                                                                        //rsinΘ,rcosΘで点の位置(ストラットの先端の位置が決まるが，ばねの最大の長さ，stripeshapeを変更しないと収縮する)
                struts[index(i, j)].name = $"Strut{index(i,j)}";                                //index関数から返された値の名前を付ける
                
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
                    spring[1].tolerance = edgeLoop[i]*0.02f;                                        //復元力が働く距離と働かない距離の誤差の設定
                    spring[1].maxDistance = edgeLoop[i];   

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
                        spring[3].tolerance = edgeLoop[i-1]*0.02f;
                        spring[3].maxDistance = edgeLoop[i-1];  

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
        }

        

    }

}
