using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

using UnityEngine.SceneManagement;

public class Searcher : MonoBehaviour
{
    

    private GameObject[] struts;
    private GameObject[] end;
    private float[] position;
    private float[] y_s;

    Assembly assembly;
    
    private float toggle =0.1f;
    private int count=0;

    // Start is called before the first frame update
    void Start()
    {   

        count=PlayerPrefs.GetInt("count");
        //countが大きければ実行
        if(count < 2.0){
            StartCoroutine(DelayMethod());
        }

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
    /*
    IEnumerator DelayMethod(){

        //edgeの長さ調整
        GameObject tm_g =GameObject.Find("tm_g");
        position =new float[8];
        
        struts = new GameObject[8];
        end = new GameObject[8];

        float tm_g_y=0.0f;
        
        
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);

        PlayerPrefs.SetFloat("StrutScale",1.0f);
        PlayerPrefs.Save();
        float edgeParameter=assembly.edgeparameter_layers;

        PlayerPrefs.SetFloat("edge",edgeParameter);
        PlayerPrefs.Save();

        //Strutの長さ調整

        yield return new WaitForSeconds(1.2f);
                

        string num1;
        string num2="Strut7";

        for(int i=2;i<10;i++){
            num1=i.ToString();
            num1=num2+num1;
            struts[i-2] = tm_g.transform.Find(num1).gameObject;
        }
        //座標集計
        for(int i=0;i<8;i++){
            end[i]=struts[i].transform.Find("end1").gameObject;
            position[i]=end[i].transform.position.y;
            
        }

        for(int i=0;i<8;i++){
            tm_g_y+=position[i];
        }


        //手先位置の計測のため，上の点の値取得
        tm_g_y=tm_g_y/8;
        float strutscale = assembly.StrutScale;
        string savedata="edgeScale," + edgeParameter.ToString() + ",strutScale,"+strutscale.ToString() + ",Average,"+tm_g_y.ToString()+  "\n";

        //Debug.Log(tm_g_y);
        //Csvの出力
        string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/edgeScale_data.csv";
        OutputCsv(path,savedata);

                
        //次の周回のstrut長さ変数の変更
        
        if(tm_g_y<=0.3f){
            edgeParameter+=0.01f;
            strutscale=1.0f;
        }
        
        else if (strutscale<=1.0f && strutscale>=0.1f){
            strutscale-=0.05f;
        }
        else if(edgeParameter<=20.0 && edgeParameter>1.0f){
            edgeParameter+=0.01f;
            strutscale=1.0f;
        }
        else{

        }
        
                
        //ストラットの長さ変数の値変更の引継ぎ
        PlayerPrefs.SetFloat("StrutScale",strutscale);
        PlayerPrefs.Save();
                
        //シーンの遷移

        

        
        PlayerPrefs.SetFloat("edge",edgeParameter);
        PlayerPrefs.Save();
        Debug.Log("edgeParameter : " +edgeParameter.ToString() +         "      strutscale : " + strutscale.ToString());
        

        if(edgeParameter>=0.0f){
            SceneManager.LoadScene("SampleScene");
        }
        
    }
    */

    IEnumerator DelayMethod(){

        //edgeの長さ調整
        GameObject tm_g =GameObject.Find("tm_g");
        position =new float[8];
        
        struts = new GameObject[8];
        end = new GameObject[8];

        //高さの取得変数
        float tm_g_y=0.0f;
        
        //Asembly.csの取得
        assembly=tm_g.GetComponent<Assembly>();
        //Debug.Log(assembly.StrutScale);

        if(PlayerPrefs.HasKey("count")){
            count=PlayerPrefs.GetInt("count");
        }
        PlayerPrefs.SetInt("count",count);
        PlayerPrefs.Save();
        
        ////////////////////////////////////////////////////////////////////////////////////////
        double psi0,phi0,phiFin,psiFin;
        
        //psiの登録
        psi0=assembly.psi0;
        PlayerPrefs.SetFloat("psi0",(float)psi0);
        psiFin=assembly.psiFin;
        PlayerPrefs.SetFloat("psifin",(float)psiFin);
        PlayerPrefs.Save();

        //phiの登録
        phi0=assembly.phi0;
        PlayerPrefs.SetFloat("phi0",(float)phi0);
        phiFin=assembly.phiFin;
        PlayerPrefs.SetFloat("phifin",(float)phiFin);
        PlayerPrefs.Save();


        //登録情報の保存
        PlayerPrefs.Save();
        ////////////////////////////////////////////////////////////////////////////////////////

        //時間の停止
        yield return new WaitForSeconds(1.2f);
                

        //高さの取得処理
        string num1;
        string num2="Strut7";

        
        y_s =new float[8];

        Debug.Log("psi0 : " +psi0.ToString() +    "psiFin : " +psiFin.ToString() +      "     phi0 : " +phi0.ToString() +    "     phiFin : " +phiFin.ToString()  +"     count : " +count );

        for(int i=2;i<10;i++){
            num1=i.ToString();
            num1=num2+num1;
            struts[i-2] = tm_g.transform.Find(num1).gameObject;
        }
        //座標集計
        for(int i=0;i<8;i++){
            end[i]=struts[i].transform.Find("end1").gameObject;
            position[i]=end[i].transform.position.y;
            
        }

        for(int i=0;i<8;i++){
            tm_g_y+=position[i];
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
        tm_g_y=tm_g_y/8;
        float strutscale = assembly.StrutScale;

        ////////////////////////////////////////////////////////////////////////////////////////

        //予測値の取得
        float predictTm_g_y=assembly.predict;
        //予測値との誤差
        double error=(double)(tm_g_y-predictTm_g_y);

        string savedata="psi0," +psi0.ToString() + ",psiFin," +psiFin.ToString() +  ",phi0," +phi0.ToString() +    ",phiFin," +phiFin.ToString()+ ",strutscale," +strutscale.ToString() + ",SimulationResult," + tm_g_y.ToString()  + ",Predict," + predictTm_g_y.ToString() + ",error," + error.ToString() + ",strtut0_y," + y_s[0] + ",strtut1_y," + y_s[1] +",strtut2_y," + y_s[2] +",strtut3_y," + y_s[3] + "\n";;
        //Debug.Log(tm_g_y);
        //Csvの出力
        string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/phi_and_psi_data_counttNo"+ count.ToString() +".csv";
        OutputCsv(path,savedata);

        


        
        //最小値のエラーの更新
        if(0.2f>=Math.Abs(error)  &&  (y_s[3]- y_s[0]) >= 0.12){
            string savedataTm_g_y="psi0," +psi0.ToString() + ",psiFin," +psiFin.ToString() +  ",phi0," +phi0.ToString() +    ",phiFin," +phiFin.ToString()+ ",strutscale," +strutscale.ToString() + ",SimulationResult," + tm_g_y.ToString()  + ",Predict," + predictTm_g_y.ToString() + ",error," + error.ToString() + ",strtut0_y," + y_s[0] + ",strtut1_y," + y_s[1] +",strtut2_y," + y_s[2] +",strtut3_y," + y_s[3] + "\n";
            //Debug.Log(tm_g_y);
            //Csvの出力
            string path2= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/Tm_g_y_GoodData_counttNo"+ count.ToString() +".csv";
            OutputCsv(path2,savedataTm_g_y);

        }

        string pathpicture="D:/picture/phi0_"+psi0.ToString()+"psiFin_"+psiFin.ToString()+"phi0_"+phi0.ToString()+"phiFin"+phiFin.ToString()+"count_"+ count.ToString() +".png";
        OnScrrenCapture(pathpicture);
        
        
                
        //次の周回のstrut長さ変数の変更
        float damper=5.0f;
        
        if(psi0<=psiFin && psi0>0.0f){
            psi0-=damper;
            
        }
        else if(psi0<=0.0f && psiFin>0.0f){
            psiFin-=damper;
            psi0=psiFin;
            
        }
        else if(phi0<=phiFin && phi0>0.0f){
            phi0-=damper;
            psiFin=90.0f;
            psi0=psiFin;
        }
        else if(phi0<=0.0f && phiFin>=0.0f){
            phiFin-=damper;
            phi0=phiFin;
            psiFin=90.0f;
            psi0=psiFin;
        }
        else if(phiFin<0.0f){
            count+=1;
            psi0=90;
            phi0=90;
            psiFin=90;
            phiFin=90;
            PlayerPrefs.SetInt("count",count);
            PlayerPrefs.Save();
        }
        
                
        //変数の値変更の引継ぎ
        PlayerPrefs.SetFloat("psi0",(float)psi0);
        PlayerPrefs.SetFloat("psifin",(float)psiFin);
        PlayerPrefs.Save();

        //phiの登録
        PlayerPrefs.SetFloat("phi0",(float)phi0);
        PlayerPrefs.SetFloat("phifin",(float)phiFin);
        PlayerPrefs.Save();

        
        
        ////////////////////////////////////////////////////////////////////////////////////////

        //シーンの遷移
        SceneManager.LoadScene("SampleScene");
        
        
    }

    

        
}

