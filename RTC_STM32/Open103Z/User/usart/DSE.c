/*-------------------------------------------------------
      Data Encryption Standard  56λ��Կ����64λ���� 
                  2011.10
--------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include "table.h"
#include "head.h"
void BitsCopy(bool *DatOut,bool *DatIn,int Len);  // ���鸴�� 

void ByteToBit(bool *DatOut,char *DatIn,int Num); // �ֽڵ�λ 
void BitToByte(char *DatOut,bool *DatIn,int Num); // λ���ֽ�

void BitToHex(char *DatOut,bool *DatIn,int Num);  // �����Ƶ�ʮ������ 64λ to 4*16�ַ�
void HexToBit(bool *DatOut,char *DatIn,int Num);  // ʮ�����Ƶ������� 

void TablePermute(bool *DatOut,bool *DatIn,const char *Table,int Num); // λ���û����� 
void LoopMove(bool *DatIn,int Len,int Num);     // ѭ������ Len���� Num�ƶ�λ�� 
void Xor(bool *DatA,bool *DatB,int Num);         // ����� 

void S_Change(bool DatOut[32],bool DatIn[48]);   // S�б任 
void F_Change(bool DatIn[32],bool DatKi[48]);    // F����                                  

void SetKey(char KeyIn[8]);                         // ������Կ
void PlayDes(char MesOut[8],char MesIn[8]);       // ִ��DES����
void KickDes(char MesOut[8],char MesIn[8]);             // ִ��DES���� 

 
char MesHex_OUT[128]={0};         // 16���ַ��������ڴ�� 64λ16���Ƶ�����
char MyMessage_Kick[125]={0};
char KEY_LOCK[9]={'1','2','3','4','5','6','7','8'}; 
char KEY_UNLOCK[9]={'1','2','3','4','5','6','7','8'}; 
void des_test(char *data,char *key_lock,char * key_unlock,unsigned int num)
{
    u8 i=0; 
    char MesHex[16]={0};         // 16���ַ��������ڴ�� 64λ16���Ƶ�����
		char MesHexr[16]={0};
    char MyKey[8]={0};           // ��ʼ��Կ 8�ֽ�*8
    char YourKey[8]={0};         // ����Ľ�����Կ 8�ֽ�*8
    char MyMessage[8]={0};       // ��ʼ���� 
    char MyMessage_OUT[8]={0};       // ��ʼ���� 
/*-----------------------------------------------*/
    u16 group,group_ero;
		u16 j=0,k=0,k1=0,l=0,l1=0;		
		group_ero=num%8;
		group=num/8;
		if(group_ero!=0)
    group++;
		
	
		for (i=0;i<8;i++){MyKey[i]=key_lock[i];YourKey[i]=key_unlock[i];}//copy key
		
		SetKey(MyKey);               // set key master
		
		
		
		for(j=0;j<group;j++)//lock
		{
		for (i=0;i<8;i++)
			MesHex[i]=0;//clear hex_buf
			
	  if(group_ero!=0&&j==group-2)
		{for (i=0;i<group_ero;i++)
			MyMessage[i]=data[l++];//copy group	
		 for (i=group_ero;i<8;i++)
			MyMessage[i]=0x00;//copy group	
		}
		else
		for (i=0;i<8;i++)
		MyMessage[i]=data[l++];//copy group
			
		PlayDes(MesHex,MyMessage);   // DSE
			
		for (i=0;i<16;i++)
		MesHex_OUT[k++]=MesHex[i];//out hex	
			
    }
    SetKey(YourKey);             // set key slave
  	for(j=0;j<group;j++)//unlock
		{
			for (i=0;i<16;i++)
			MesHexr[i]=MesHex_OUT[k1++];//out hex	
       KickDes(MyMessage_OUT,MesHexr);                     // ���������MyMessage   
      for (i=0;i<8;i++)
			MyMessage_Kick[l1++]=MyMessage_OUT[i];//copy group
		}
}


void des_unlock(char *data,char *key_lock,char * key_unlock,unsigned int num,u8 en)
{
    u8 i=0; 
    char MesHex[16]={0};         // 16���ַ��������ڴ�� 64λ16���Ƶ�����
		char MesHexr[16]={0};
    char MyKey[8]={0};           // ��ʼ��Կ 8�ֽ�*8
    char YourKey[8]={0};         // ����Ľ�����Կ 8�ֽ�*8
    char MyMessage[8]={0};       // ��ʼ���� 
    char MyMessage_OUT[8]={0};       // ��ʼ���� 
/*-----------------------------------------------*/
    u16 group,group_ero;
		u16 j=0,k=0,k1=0,l=0,l1=0;		
		group_ero=num%8;
		group=num/8;
		if(group_ero!=0)
    group++;
		
	
		for (i=0;i<8;i++){MyKey[i]=key_lock[i];YourKey[i]=key_unlock[i];}//copy key
		
		SetKey(MyKey);               // set key master
		
		
		
		for(j=0;j<group;j++)//lock
		{
		for (i=0;i<8;i++)
			MesHex[i]=0;//clear hex_buf
			
	  if(group_ero!=0&&j==group-2)
		{for (i=0;i<group_ero;i++)
			MyMessage[i]=data[l++];//copy group	
		 for (i=group_ero;i<8;i++)
			MyMessage[i]=0x00;//copy group	
		}
		else
		for (i=0;i<8;i++)
		MyMessage[i]=data[l++];//copy group
			
	//	PlayDes(MesHex,MyMessage);   // DSE
			
		for (i=0;i<16;i++)
		{MesHex_OUT[k]=data[k];k++;}//out hex	
			
    }
		if(en){
    SetKey(YourKey);             // set key slave
  	for(j=0;j<group;j++)//unlock
		{
			for (i=0;i<16;i++)
			MesHexr[i]=MesHex_OUT[k1++];//out hex	
       KickDes(MyMessage_OUT,MesHexr);                     // ���������MyMessage   
      for (i=0;i<8;i++)
			MyMessage_Kick[l1++]=MyMessage_OUT[i];//copy group
		}
	}
	else
  	for(j=0;j<group;j++)//unlock
		{                // ���������MyMessage   
      for (i=0;i<8;i++)
			{MyMessage_Kick[l1]=MesHex_OUT[l1];l1++;}//copy group
		}	
}
/*-------------------------------
 ��DatIn��ʼ�ĳ���λLenλ�Ķ�����
 ���Ƶ�DatOut��
--------------------------------*/
void BitsCopy(bool *DatOut,bool *DatIn,int Len)     // ���鸴�� OK 
{
    int i=0;
    for(i=0;i<Len;i++)
    {
        DatOut[i]=DatIn[i];
    }
}

/*-------------------------------
 �ֽ�ת����λ���� 
 ÿ8�λ�һ���ֽ� ÿ��������һλ
 ��1��ȡ���һλ ��64λ 
--------------------------------*/
void ByteToBit(bool *DatOut,char *DatIn,int Num)       // OK
{
    int i=0;
    for(i=0;i<Num;i++)
    {
        DatOut[i]=(DatIn[i/8]>>(i%8))&0x01;   
    }                                       
}

/*-------------------------------
 λת�����ֽں���
 �ֽ�����ÿ8����һλ
 λÿ�������� ����һ�λ�   
---------------------------------*/
void BitToByte(char *DatOut,bool *DatIn,int Num)        // OK
{
    int i=0;
    for(i=0;i<(Num/8);i++)
    {
        DatOut[i]=0;
    } 
    for(i=0;i<Num;i++)
    {
        DatOut[i/8]|=DatIn[i]<<(i%8);    
    }        
}


/*----------------------------------
 ����������ת��Ϊʮ������
 ��Ҫ16���ַ���ʾ
-----------------------------------*/
void BitToHex(char *DatOut,bool *DatIn,int Num)
{
    int i=0;
    for(i=0;i<Num/4;i++)
    {
        DatOut[i]=0;
    }
    for(i=0;i<Num/4;i++)
    {
        DatOut[i] = DatIn[i*4]+(DatIn[i*4+1]<<1)
                    +(DatIn[i*4+2]<<2)+(DatIn[i*4+3]<<3);
        if((DatOut[i]%16)>9)
        {
            DatOut[i]=DatOut[i]%16+'7';       //  ��������9ʱ���� 10-15 to A-F
        }                                     //  ����ַ� 
        else
        {
            DatOut[i]=DatOut[i]%16+'0';       //  ����ַ�       
        }
    }
    
}

/*---------------------------------------------
 ʮ�������ַ�ת������
----------------------------------------------*/
void HexToBit(bool *DatOut,char *DatIn,int Num)
{
    int i=0;                        // �ַ������� 
    for(i=0;i<Num;i++)
    {
        if((DatIn[i/4])>'9')         //  ����9 
        {
            DatOut[i]=((DatIn[i/4]-'7')>>(i%4))&0x01;               
        }
        else
        {
            DatOut[i]=((DatIn[i/4]-'0')>>(i%4))&0x01;     
        } 
    }    
}

// ���û�����  OK
void TablePermute(bool *DatOut,bool *DatIn,const char *Table,int Num)  
{
    int i=0;
    static bool Temp[256]={0};
    for(i=0;i<Num;i++)                // NumΪ�û��ĳ��� 
    {
        Temp[i]=DatIn[Table[i]-1];  // ԭ�������ݰ���Ӧ�ı��ϵ�λ������ 
    }
    BitsCopy(DatOut,Temp,Num);       // �ѻ���Temp��ֵ��� 
} 

// ����Կ����λ
void LoopMove(bool *DatIn,int Len,int Num) // ѭ������ Len���ݳ��� Num�ƶ�λ��
{
    static bool Temp[256]={0};    // ����   OK
    BitsCopy(Temp,DatIn,Num);       // ����������ߵ�Numλ(���Ƴ�ȥ��)����Temp 
    BitsCopy(DatIn,DatIn+Num,Len-Num); // ��������߿�ʼ�ĵ�Num����ԭ���Ŀռ�
    BitsCopy(DatIn+Len-Num,Temp,Num);  // ���������Ƴ�ȥ�����ݼӵ����ұ� 
} 

// ��λ���
void Xor(bool *DatA,bool *DatB,int Num)           // �����
{
    int i=0;
    for(i=0;i<Num;i++)
    {
        DatA[i]=DatA[i]^DatB[i];                  // ��� 
    }
} 

// ����48λ ���32λ ��Ri���
void S_Change(bool DatOut[32],bool DatIn[48])     // S�б任
{
    int i,X,Y;                                    // iΪ8��S�� 
    for(i=0,Y=0,X=0;i<8;i++,DatIn+=6,DatOut+=4)   // ÿִ��һ��,��������ƫ��6λ 
    {                                              // ÿִ��һ��,�������ƫ��4λ
        Y=(DatIn[0]<<1)+DatIn[5];                          // af����ڼ���
        X=(DatIn[1]<<3)+(DatIn[2]<<2)+(DatIn[3]<<1)+DatIn[4]; // bcde����ڼ���
        ByteToBit(DatOut,&S_Box[i][Y][X],4);      // ���ҵ��ĵ����ݻ�Ϊ������    
    }
}

// F����
void F_Change(bool DatIn[32],bool DatKi[48])       // F����
{
    static bool MiR[48]={0};             // ����32λͨ��Eѡλ��Ϊ48λ
    TablePermute(MiR,DatIn,E_Table,48); 
    Xor(MiR,DatKi,48);                   // ������Կ���
    S_Change(DatIn,MiR);                 // S�б任
    TablePermute(DatIn,DatIn,P_Table,32);   // P�û������
}



void SetKey(char KeyIn[8])               // ������Կ ��ȡ����ԿKi 
{
    int i=0;
    static bool KeyBit[64]={0};                // ��Կ�����ƴ洢�ռ� 
    static bool *KiL=&KeyBit[0],*KiR=&KeyBit[28];  // ǰ28,��28��56
    ByteToBit(KeyBit,KeyIn,64);                    // ����ԿתΪ�����ƴ���KeyBit 
    TablePermute(KeyBit,KeyBit,PC1_Table,56);      // PC1���û� 56��
    for(i=0;i<16;i++)
    {
        LoopMove(KiL,28,Move_Table[i]);       // ǰ28λ���� 
        LoopMove(KiR,28,Move_Table[i]);          // ��28λ���� 
         TablePermute(SubKey[i],KeyBit,PC2_Table,48); 
         // ��ά���� SubKey[i]Ϊÿһ����ʼ��ַ 
         // ÿ��һ��λ����PC2�û��� Ki 48λ 
    }        
}

void PlayDes(char MesOut[8],char MesIn[8])  // ִ��DES����
{                                           // �ֽ����� Bin���� Hex��� 
    int i=0;
    static bool MesBit[64]={0};        // ���Ķ����ƴ洢�ռ� 64λ
    static bool Temp[32]={0};
    static bool *MiL=&MesBit[0],*MiR=&MesBit[32]; // ǰ32λ ��32λ 
    ByteToBit(MesBit,MesIn,64);                 // �����Ļ��ɶ����ƴ���MesBit
    TablePermute(MesBit,MesBit,IP_Table,64);    // IP�û� 
    for(i=0;i<16;i++)                       // ����16�� 
    {
        BitsCopy(Temp,MiR,32);            // ��ʱ�洢
        F_Change(MiR,SubKey[i]);           // F�����任
        Xor(MiR,MiL,32);                  // �õ�Ri 
        BitsCopy(MiL,Temp,32);            // �õ�Li 
    }                    
    TablePermute(MesBit,MesBit,IPR_Table,64);
    BitToHex(MesOut,MesBit,64);
}

void KickDes(char MesOut[8],char MesIn[8])       // ִ��DES����
{                                                // Hex���� Bin���� �ֽ���� 
    int i=0;
    static bool MesBit[64]={0};        // ���Ķ����ƴ洢�ռ� 64λ
    static bool Temp[32]={0};
    static bool *MiL=&MesBit[0],*MiR=&MesBit[32]; // ǰ32λ ��32λ
    HexToBit(MesBit,MesIn,64);                 // �����Ļ��ɶ����ƴ���MesBit
    TablePermute(MesBit,MesBit,IP_Table,64);    // IP�û� 
    for(i=15;i>=0;i--)
    {
        BitsCopy(Temp,MiL,32);
        F_Change(MiL,SubKey[i]);
        Xor(MiL,MiR,32);
        BitsCopy(MiR,Temp,32);
    }    
    TablePermute(MesBit,MesBit,IPR_Table,64);
    BitToByte(MesOut,MesBit,64);        
} 