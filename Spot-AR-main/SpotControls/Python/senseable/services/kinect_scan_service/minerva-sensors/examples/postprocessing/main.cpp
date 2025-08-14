#include"cnpy.h"
#include<complex>
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>

#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "FlirBosonRawCountsToTemperatureConversion.h"
#include <fstream>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <filesystem>


const size_t Nx = 128;
const size_t Ny = 64;
const size_t Nz = 32;



// AGC Sample ONE: Linear from min to max.
// Input is a MATRIX (height x width) of 16bits. (OpenCV mat)
// Output is a MATRIX (height x width) of 8 bits (OpenCV mat)
void AGC_Basic_Linear(cv::Mat input_16, cv::Mat output_8, int height, int width) {
    int i, j;  // aux variables

    // auxiliary variables for AGC calcultion
    unsigned int max1=0;         // 16 bits
    unsigned int min1=0xFFFF;    // 16 bits
    unsigned int value1, value2, value3, value4;

    // RUN a super basic AGC
    for (i=0; i<height; i++) {
        for (j=0; j<width; j++) {
            value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
            value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
            value3 = ( value1 << 8) + value2;
            if ( value3 <= min1 ) {
                min1 = value3;
            }
            if ( value3 >= max1 ) {
                max1 = value3;
            }
            //printf("%X.%X.%X  ", value1, value2, value3);
        }
    }
    //printf("max1=%04X, min1=%04X\n", max1, min1);

    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            value1 =  input_16.at<uchar>(i,j*2+1) & 0XFF ;  // High Byte
            value2 =  input_16.at<uchar>(i,j*2) & 0xFF  ;    // Low Byte
            value3 = ( value1 << 8) + value2;
            value4 = ( ( 255 * ( value3 - min1) ) ) / (max1-min1)   ;
            // printf("%04X \n", value4);

            output_8.at<uchar>(i,j)= (uchar)(value4&0xFF);
        }
    }

}

// Comparator Function
bool myCmp(cv::String s1, cv::String s2)
{
 
    // If size of numeric strings
    // are same the put lowest value
    // first
    if (s1.size() == s2.size()) {
        return s1 < s2;
    }
 
    // If size is not same put the
    // numeric string with less
    // number of digits first
    else {
        return s1.size() < s2.size();
    }
}

// gets the temperatures from the raw image and puts them into python mat files
int temperatureToPythonFiles(uint32_t camera_sn, cv::String path) {
    // start of ICI code
    cv::Mat thermalFrame; // gray16 image
    uint16_t fpa; // focal plane area
    char filename[120]; // filename of the tiff image
    char txtname[120]; // name of the txt file that holds the fpa value
    char npyname[120]; // name of the npy file
    char foldername[40]; // foldername for the processed files

    std::string currpath = std::filesystem::current_path().string();
    std::string binpath = currpath;

    // remove the last segment of the current path and then append the bin directory
    std::size_t found = binpath.find_last_of("/\\");
    binpath = binpath.substr(0,found);
    binpath.append("/bin");


    int status = 0; // to hold status of chdir function

    // get the handle to enter into the loop
    auto const handle = ICICreateFlirBosonRawCountsToTemperatureInCelsiusHandle();
    if (handle)
    {
        auto const calibrationFileDirectoryPath = binpath.c_str();
        if (ICILoadCalibrationFileData(handle, calibrationFileDirectoryPath, camera_sn))
        {
            // preliminary variables
            int rows = 512;
            int cols = 640;
            int totalSize = rows * cols;

            //find number of images
            std::string cameraSNtmp = std::to_string(camera_sn);
            cameraSNtmp.insert(0, "_");

            size_t count = 0;
            // get the number of files in the folder
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                if (cv::String(entry.path()).find(cameraSNtmp) != std::string::npos) { // checks for files for the camera
                    if (cv::String(entry.path()).find(".tiff") != std::string::npos) { // then makes sure it is a tiff file
                        count++;
                    }
                }
            }

            if (count == 0) { // exit if no frames are available for that camera
                std::cout << "No frames found from camera " << camera_sn << std::endl;
                return 0;
            }

             // create folder for the camera and create it if it doesnt already exist
            sprintf(foldername, "ProcessedImages/cap%d/npy/", camera_sn);
            mkdir(foldername, 0700);
            

            const char* path2 = path.c_str(); // sprintf doesn't work well with strings
            const char* path3 = currpath.c_str(); // cstring for the current path


            // load in image data
            for (size_t i=0; i<count; i++) {
                sprintf(filename, "%s/%lu-raw16_%d.tiff", path2, i, camera_sn);

                thermalFrame = cv::imread(filename, cv::IMREAD_ANYDEPTH);

                // load in FPA value from file
                sprintf(txtname, "%s/%lu-raw16_%d.txt", path2, i, camera_sn);
                std::ifstream myfile (txtname);

                if ( myfile.is_open() ) { // always check whether the file is open
                    myfile >> fpa; // pipe file's content into stream
                }

                myfile.close();

                auto temperatureInCeslius = ICIConvertFlirBosonCameraRawCountsToTemperatureInCelsius(
                    handle, (uint16_t *)thermalFrame.data, totalSize, fpa);
                if (temperatureInCeslius){
                    // print to file
                    sprintf(npyname, "%s/%s%lu-raw16_%d.npy", path3, foldername, i, camera_sn);
                    std::string npyname2(npyname);
                    cnpy::npy_save(npyname2, &temperatureInCeslius[0], {512,640}, "w");
                }
            } 
        }
        ICIDestroyFlirBosonRawCountsToTemperatureInCelsiusHandle(handle);
    }
    return 1;
}

//processes the images for one camera
size_t processImages(cv::String cameraSN, cv::String path, char colormap) {
    char imageName[60]; //image name for the processed images
    char imageName2[60]; // image name for the raw files to be resaved into the processed folder
    char foldername[30]; //foldername for the agc images
    char foldernameraw[30]; //foldername for the raw images
    char filename[120]; // filename of the image
    int status = 0;
    std::string cameraSNtmp = cameraSN;
    cameraSNtmp.insert(0, "_");

    size_t count = 0;
    // get the number of files in the folder
    for (const auto& entry : std::filesystem::directory_iterator(path)) {
        if (cv::String(entry.path()).find(cameraSNtmp) != std::string::npos) { // checks for files for the camera
            if (cv::String(entry.path()).find(".tiff") != std::string::npos) { // then makes sure it is a tiff file
                count++;
            }
        }
    }

    if (count == 0) { // exit if no frames are available for that camera
        std::cout << "No frames found from camera " << cameraSN << std::endl;
        return 0;
    }

    cv::Mat image; // mat to hold the image

    // create folder for the camera and create it if it doesnt already exist
    sprintf(foldername, "ProcessedImages");
    mkdir(foldername, 0700);
    status = chdir(foldername);
    if (status != 0) {
        std::cout << "Error changing directories \n";
        std::abort();
    }

    const char* path2 = path.c_str(); // sprintf doesn't work well with strings
    const char* cameraSN2 = cameraSN.c_str();

    // loop through the frames using the known naming convention of framecount-raw16_cameraSN.tiff

    for (size_t i=0; i<count; i++) {
        sprintf(filename, "%s/%lu-raw16_%s.tiff", path2, i, cameraSN2);

        image = cv::imread(filename, cv::IMREAD_ANYDEPTH);
        int width;
        int height;

        height = image.rows;
        width = image.cols;
        cv::Mat gray8(height, width, CV_8U, 1); // create buffer for the new image


        cv::String filenameS(filename); // turn the filename into a string in order to perform string functions on it

        // get the number of the frame
        int start = filenameS.find_last_of('/') + 1;
        int last = filenameS.find_last_of('-') - 1;
        int length = last - start + 1;

        sprintf(foldername, "cap%s", cameraSN2);
        mkdir(foldername, 0700);
        sprintf(foldername, "cap%s/agc", cameraSN2);
        mkdir(foldername, 0700);

        // make folder for the raw files
        sprintf(foldernameraw, "cap%s/raw", cameraSN2);
        mkdir(foldernameraw, 0700);

        AGC_Basic_Linear(image, gray8, height, width);
        // save linear version with colormaps if desired
        sprintf(imageName, "%s/%d-agc_%s.tiff", foldername, stoi(filenameS.substr(start, length)), cameraSN2); // file names are not in order so the index needs to be retrieved from the filename
        sprintf(imageName2, "%s/%d-raw16_%s.tiff", foldernameraw, stoi(filenameS.substr(start, length)), cameraSN2); // file names are not in order so the index needs to be retrieved from the filename


        if( colormap == 'i') {
            cv::cvtColor(gray8, gray8, cv::COLOR_GRAY2BGR, 3);
            cv::applyColorMap(gray8, gray8, cv::COLORMAP_INFERNO);
        }
        cv::imwrite(imageName, gray8);
        cv::imwrite(imageName2, image);
    }

    status = chdir("..");
    if (status != 0) {
        std::cout << "Error changing directories \n";
        std::abort();
    }
    printf("Images from camera %s processed \n", cameraSN2);

    return count;
}


// create a video of the frames in a folder
void makeVideo(size_t count, cv::String cameraSN) {
    // Path to the directory
    std::string path = std::filesystem::current_path().string();
 

    // add the cszamera directory to path
    path.append("/ProcessedImages/cap");
    path.append(cameraSN);

    // Testing whether the path points to a directory
    if (std::filesystem::is_directory(path)) {
        std ::cout << path << std::endl;

        //get the images
        // outfilename_str.append("/*.tiff");
        char subfolderFile[120];

        char videoName[120];
        sprintf(videoName, "%s/%s.avi", path.c_str(), cameraSN.c_str());

        // create videowriter object
        cv::VideoWriter video(videoName, cv::CAP_FFMPEG, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(640, 512), 0);

        // framecount-acg_cameraSN.tiff
        for (size_t i = 0; i < count; i++) {
            //read in images
            cv::Mat frame;

            // get frame
            sprintf(subfolderFile, "%s/agc/%lu-agc_%s.tiff", path.c_str(), i, cameraSN.c_str());
            frame = cv::imread(subfolderFile, cv::IMREAD_ANYDEPTH);
            if (frame.empty())
                std::cout << "Image File " << subfolderFile << "Not Found" << std::endl;

            // write frame to video
            video.write(frame);
        }

        // release video
        video.release();
    }
}


// takes an absolute path to a folder as an argument 
// it will then run an AGC on all of the tiff images in the folder
// the postprocessed images end up in a 'processedImages' folder
int main(int argc, char** argv) {
    cv::String pattern = "";
    size_t count = 0;
    char colormap = 'w';
    // Read command line arguments
    for (int i=0; i < argc; i++) {
    
        // get foldername
		if ( argv[i][0]=='/') {
			pattern = argv[i];
		}    

        // get colormap
        if ( argv[i][0]=='i') {
            colormap = 'i';
        }      
    }

    //check if a pattern was passed in as an argument and give an opportunity to provide one if not
    while (pattern.length() < 1) {
        std::cout << "Folder path is not long enough \n \n Enter path: ";
        std::cin >> pattern;
    }

    // // for testing
    // pattern = "/home/sdl/Documents/minerva-sensors/build/BosonImages";

    // get the pattern for each of the 6 cameras
    /*
    221687
    221013
    221017
    221021
    221029
    221030
    */

   // *_221030

    if (pattern.back() == '/') {
        pattern.pop_back();
    } 
    count = processImages("221030", pattern, colormap);
    if (count > 0) {
        temperatureToPythonFiles(221030, pattern);
        printf("Starting to create video \n");
        makeVideo(count, "221030");
    }

    // *_221029

    if (pattern.back() == '/') {
        pattern.pop_back();
    } 
    count = processImages("221029", pattern, colormap);
    if (count > 0) {
        temperatureToPythonFiles(221029, pattern);
        printf("Starting to create video \n");
        makeVideo(count, "221029");
    }


    // *_221021

    if (pattern.back() == '/') {
        pattern.pop_back();
    } 
    count = processImages("221021", pattern, colormap);
    if (count > 0) {
        temperatureToPythonFiles(221021, pattern);
        printf("Starting to create video \n");
        makeVideo(count, "221021");
    }


    // *_221017

    if (pattern.back() == '/') {
        pattern.pop_back();
    } 
    count = processImages("221017", pattern, colormap);
    if (count > 0) {
        temperatureToPythonFiles(221017, pattern);
        printf("Starting to create video \n");
        makeVideo(count, "221017");
    }


    // *_221013
    
    if (pattern.back() == '/') {
        pattern.pop_back();
    } 
    count = processImages("221013", pattern, colormap);
    if (count > 0) {
        temperatureToPythonFiles(221013, pattern);
        printf("Starting to create video \n");
        makeVideo(count, "221013");
    }


    // *_221687

    if (pattern.back() == '/') {
        pattern.pop_back();
    } 
    count = processImages("221687", pattern, colormap);
    if (count > 0) {
        temperatureToPythonFiles(221687, pattern);
        printf("Starting to create video \n");
        makeVideo(count, "221687");
    }






    //set random seed so that result is reproducible (for testing)
    srand(0);
    //create random data
    std::vector<std::complex<double>> data(Nx*Ny*Nz);
    for(int i = 0;i < Nx*Ny*Nz;i++) data[i] = std::complex<double>(rand(),rand());
    // this does nothing but it keeps the file from crashing on compilation
    cnpy::npz_save("out.npz","arr1",&data[0],{Nz,Ny,Nx},"w"); 






    return 0;
};