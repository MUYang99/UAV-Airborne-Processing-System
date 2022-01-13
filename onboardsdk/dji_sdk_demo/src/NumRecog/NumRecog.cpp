#include "NumRecog.h"

NumRecog::NumRecog()
{
}

NumRecog::~NumRecog()
{
    svm_free_model_content(svm);
}

void NumRecog::loadModel(const char *model_file_name)
{
    svm = svm_load_model(model_file_name);
}

int NumRecog::getNumber(Mat& image)
{
    
	/*IplImage* testImage;
	IplImage  testImage0;
	IplImage* resizeImage;

	testImage0 = image.operator _IplImage();
	testImage = &testImage0;

	if( testImage == NULL )
	{
        cout<<" can not load the image !"<<endl;
        return -1;
	}
	resizeImage =cvCreateImage(cvSize(20,20),8,1);
	cvZero(resizeImage);

	cvResize(testImage,resizeImage);*/
    
    Mat resizeMat(20,20,CV_8UC1);
    cv::resize(image, resizeMat, cv::Size(20, 20));
    
	cv::HOGDescriptor *hog=new cv::HOGDescriptor(cvSize(20,20),cvSize(10,10),cvSize(5,5),cvSize(5,5),9);
	vector<float>descriptors;
    hog->compute(resizeMat, descriptors,Size(1,1), Size(0,0));
    

	svm_node * inputVector = new svm_node [ descriptors.size()+1];
	int n = 0;
	for(vector<float>::iterator iter=descriptors.begin();iter!=descriptors.end();iter++)
	{
	  inputVector[n].index = n;
	  inputVector[n].value = *iter;
	  n++;
	}
	inputVector[n].index = -1;

	return svm_predict(svm,inputVector);
}




char* NumRecog::readline(FILE *input)
{
    int len;

    if (fgets(tem_line, max_line_len, input) == NULL)
            return NULL;

    while (strrchr(tem_line, '\n') == NULL)
    {
        max_line_len *= 2;
        tem_line = (char *)realloc(tem_line, max_line_len);
        len = (int)strlen(tem_line);
        if (fgets(tem_line + len, max_line_len - len, input) == NULL)
                break;
    }
    return tem_line;
}



void NumRecog::readTrainFileList(vector<string>& trainImageList,  vector<int>& trainLabelList)
{
    FILE *input1, *input2;
    max_line_len = 1024;
    tem_line = (char *)malloc(max_line_len*sizeof(char));

    input1 = fopen(trainImageFile, "r");
    input2 = fopen(testImageFile, "r");
    string value1;
    char *tem;
    string value2;
    while (readline(input1)!=NULL)
    {
        value1 = strtok(tem_line, "\n");
        trainImageList.push_back( value1 );//???¡???
    }

    while (readline(input2)!=NULL)
    {
        int i = 1;
        while(1)
        {
            if(i == 1)
                tem = strtok(tem_line, "\t");
            else
                tem = strtok(NULL, "\t");
            if(tem==NULL)
                break;
            value2 = tem;
            int label = atoi(value2.c_str());
            trainLabelList.push_back( label );//???¡???
            i++;
            cout<<label<<endl;
        }
        cout<<"run!"<<endl;
    }
    fclose(input1);
    fclose(input2);
    cout<<"Read Train Data Complete"<<endl;
}

//***************************************************************
// ????:    processHogFeature
// ????:    ????Hog????
// ???:    public
// ?????:  void
// ????:    vector<string> trainImageList
// ????:    vector<int> trainLabelList
// ????:    CvMat *  & dataMat
// ????:    CvMat *  & labelMat
//***************************************************************
void NumRecog::processHogFeature(vector<string> trainImageList,vector<int> trainLabelList, CvMat * &dataMat,CvMat * &labelMat)
{

        int trainSampleNum = trainImageList.size();
        dataMat = cvCreateMat( trainSampleNum, 900, CV_32FC1 );  //324?Hog feature Size
        cvSetZero( dataMat );
        labelMat = cvCreateMat( trainSampleNum, 1, CV_32FC1 );
        cvSetZero( labelMat );
        IplImage* src;
        IplImage* trainImg=cvCreateImage(cvSize(30,30),8,1);//20 20

        for( int i = 0; i != trainImageList.size(); i++ )
        {

            Mat image;
            char filename[200];
            sprintf(filename, "/home/exbot/Downloads/Recognization_Num/????/trainImage_%d.jpg", i+1);
            image = imread(filename, 0);
            cout<<" now is  "<<i+1<<endl;
            /*IplImage  testImage0;
            testImage0 = image.operator _IplImage();
            src = &testImage0;
            if( src == NULL )
            {
                cout<<" can not load the image: "<<(trainImageList[i]).c_str()<<endl;
                continue;
            }
            cvResize(src,trainImg);*/
            
            Mat trainImg(20,20,CV_8UC1);
            cv::resize(image, trainImg, cv::Size(20, 20));
            
            cv::HOGDescriptor *hog=new cv::HOGDescriptor(cvSize(30,30),cvSize(10,10),cvSize(5,5),cvSize(5,5),9);
            vector<float>descriptors;
            hog->compute(trainImg, descriptors,Size(1,1), Size(0,0));

            int j =0;
            for(vector<float>::iterator iter=descriptors.begin();iter!=descriptors.end();iter++)
            {
                    cvmSet(dataMat,i,j,*iter);//?›¥HOG????
                    j++;
            }
            cout<<"size: "<<j<<endl;
            cout<<trainLabelList[i]<<endl;
            cvmSet( labelMat, i, 0, trainLabelList[i] );
        }
        cout<<"Calculate Hog Feature Complete"<<endl;
        cout<<dataMat<<endl;
}

//***************************************************************
// ????:    trainLibSVM
// ????:    ????????LibSVM????SVM???
// ???:    public
// ?????:  void
// ????:    CvMat * & dataMat
// ????:    CvMat *  & labelMat
//***************************************************************
void NumRecog::trainLibSVM(CvMat *& dataMat, CvMat * & labelMat)
{
        cout<<"LibSVM start"<<endl;
        //????SVM????
        svm_parameter param;
        // default values
        param.svm_type = C_SVC;
        param.kernel_type = RBF;
        param.degree = 10;
        param.gamma = 0.0078125;	// 1/num_features
        param.coef0 = 0;
        param.nu = 0.5;
        param.cache_size = 100;
        param.C = 32.0;
        param.eps = 1e-3;
        param.p = 0.1;
        param.shrinking = 1;
        param.probability = 0;
        param.nr_weight = 0;
        param.weight_label = NULL;
        param.weight = NULL;

        //svm_prob???
        svm_problem svm_prob;

        int sampleNum = dataMat->rows;
        int vectorLength = dataMat->cols;

        svm_prob.l = sampleNum;
        svm_prob.y = new double [sampleNum];

        for (int i = 0; i < sampleNum; i++)
        {
                svm_prob.y[i] = cvmGet(labelMat,i,0);
        }

        cout<<"LibSVM middle"<<endl;
        svm_prob.x = new  svm_node * [sampleNum];

        ofstream outfile1;
        outfile1.open("train_scaled.txt");

        for (int i = 0; i < sampleNum; i++)
        {

            //fprintf(stderr, "%d\n", i);
            outfile1 << svm_prob.y[i];
            outfile1 << "\t";

            svm_node * x_space = new svm_node [vectorLength + 1];
            for (int j = 0; j < vectorLength; j++)
            {
                x_space[j].index = j;
                x_space[j].value = cvmGet(dataMat,i,j);
                outfile1 << j + 1;
                outfile1 << ":";
                outfile1 << x_space[j].value;
                outfile1 << "\t";
            }
            outfile1 << "\n";
            x_space[vectorLength].index = -1;//???????????????????????

            svm_prob.x[i] = x_space;
        }
        outfile1.close();
        cout<<"libsvm begin to train"<<endl;
        svm_model * svm_model = svm_train(&svm_prob, &param);
        cout<<"libsvm has been trained"<<endl;

        svm_save_model("MyLibsvmModel.model",svm_model);

        for (int i=0 ; i < sampleNum; i++)
        {
                delete [] svm_prob.x[i];
        }

        delete [] svm_prob.y;
        svm_free_model_content(svm_model);
}


void NumRecog::trainmodel()
{
    vector<string> trainImageList;
    vector<int> trainLabelList;
    vector<string> testImageList;
    
    readTrainFileList(trainImageList, trainLabelList);
    processHogFeature(trainImageList, trainLabelList, dataMat,labelMat);
    //trainSVM(dataMat,labelMat );
    //processNonFeature(trainImageList,trainLabelList, dataMat,labelMat);
    trainLibSVM(dataMat,labelMat);
    //readTestFileList( testImageFile,  testBasePath, testImageList);
    //testLibSVM("/home/kaiwang/SVMMinistRecognization/libsvm_hog_minist_model.model",testImageList,SVMModel);
    //testSVM( testImageList, SVMModel);
    //releaseAll();
}



