using UnityEngine;
using System;
using System.IO.Ports;
using System.Collections;

public class Rotator2 : MonoBehaviour
{
	SerialPort serial = new SerialPort ("COM4", 115200);

	//Declaração de variáveis
	public string ArduinoRead="";
	public GameObject firstBone1, firstBone2, firstBone3, firstBone4, firstBone5;
	public GameObject middleBone1, middleBone2, middleBone3, middleBone4, middleBone5;
	public GameObject lastBone1, lastBone2, lastBone3, lastBone4, lastBone5;
	public GameObject hand, arm;
	public Vector3 rotacaoMao, rotacaoBraco, rotacaoFinal;
	public string[] Output;
	float w, x, y, z, w2, x2, y2, z2;
	Quaternion quaternionBraco, quaternionMao, quaternionFinal;
    
	//Configuração da conexão da porta
	void Start ()
	{
		if (serial.IsOpen) {
			serial.Close ();
		}
		serial.Open ();
		serial.ReadTimeout = 100;
	}

	//Função Update chamada a cada atualização de frame
	void Update ()
	{
		//verifica se a porta serial está aberta
		if (serial.IsOpen) {
			
			try {
				//envia caractere pela porta serial, indicando que estamos prontos para atualizar as posições
				serial.Write("o");
				//lê contaúdo da poorta serial
				ArduinoRead = serial.ReadLine ();
			} catch (System.Exception e) {
				Debug.LogException (e);
			}

			//separação dos valores da string
			Output = ArduinoRead.Split (new char[] { ';' }, StringSplitOptions.RemoveEmptyEntries);
		
			//atualização das posições dos dedos
			//Polegar
			firstBone1.transform.localEulerAngles = new Vector3 (int.Parse (Output [10])-50, 225, 90);
			middleBone1.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [0]));
			lastBone1.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [1]));

			//Indicador
			firstBone2.transform.localEulerAngles = new Vector3 (0, -int.Parse (Output [11]), int.Parse (Output [2]));
			middleBone2.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [3]));
			lastBone2.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [3]) * 2 / 3);

			//Médio
			firstBone3.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [4]));
			middleBone3.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [5]));
			lastBone3.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [5]) * 2 / 3);

			//Anelar
			firstBone4.transform.localEulerAngles = new Vector3 (0, int.Parse (Output [12]), int.Parse (Output [6]));
			middleBone4.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [7]));
			lastBone4.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [7]) * 2 / 3);

			//Mínino
			firstBone5.transform.localEulerAngles = new Vector3 (0, int.Parse (Output [13])+int.Parse (Output [12])-10, int.Parse (Output [8]));
			middleBone5.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [9]));
			lastBone5.transform.localEulerAngles = new Vector3 (0, 0, int.Parse (Output [9]) * 2 / 3);

			//variáveis dos quaterniões da mão e do braço
			w = float.Parse (Output [14]);
			x = float.Parse (Output [15]);
			y = float.Parse (Output [16]);
			z = float.Parse (Output [17]);
			w2 = float.Parse (Output [18]);
			x2 = float.Parse (Output [19]);
			y2 = float.Parse (Output [20]);
			z2 = float.Parse (Output [21]);

			//atribuição dos objetos dos quaterniões de acordo com as variáveis anteriores
			quaternionBraco.Set (x, -z, y, w);
			quaternionMao.Set (x2, -z2, y2, w2);

			//cálculo da direfença entre os ângulos de orientação
			rotacaoFinal = quaternionMao.eulerAngles-quaternionBraco.eulerAngles;

            //permite rotação do braço apenas em torno do eixo X
            quaternionBraco.Set (x, 0, 0, w);
            
            //Atualiza as rotações do braço e da mão
            arm.transform.rotation = quaternionBraco;
            hand.transform.localEulerAngles = rotacaoFinal;
		}
	}

	void OnApplicationQuit ()
	{
		//fecha porta serial ao sair da aplicação
		serial.Close ();
	}
}