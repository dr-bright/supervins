#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <DBoW3.h>

#include "extractor_matcher_dpl.h" 
#include "transform_dpl.h"

using namespace std;
using namespace DBoW3;

void loadImages(const string &dir, vector<string> &image_paths) {
    cv::glob(dir + "/*.png", image_paths, false);
    if (image_paths.empty())
        cv::glob(dir + "/*.jpg", image_paths, false);
    if (image_paths.empty())
        cv::glob(dir + "/*.jpeg", image_paths, false);
}

int main(int argc, char **argv) {
    if(argc < 3) {
        cerr << "Usage: " << argv[0] << " <superpoint_onnx_path> <image_directory1> <image_directory2> <image_directoryN>" << endl;
        return 1;
    }

    string onnx_path = argv[1];
    vector<string> image_dirs;
    for(int i = 2; i < argc; i++) {
        image_dirs.push_back(argv[i]);
    }

    vector<string> images;
    for(const auto &image_dir : image_dirs) {
        loadImages(image_dir, images);
    }

    if (images.empty()) {
        cerr << "No images found in any of the directories." << endl;
        return 1;
    }

    cout << "Found " << images.size() << " images." << endl;

    const int MAX_IMAGES = 8000;
    if(images.size() > MAX_IMAGES) {
        random_shuffle(images.begin(), images.end());
        images.resize(MAX_IMAGES);
        cout << "Subsampled to " << MAX_IMAGES << " images for training." << endl;
    }

    // Initialize Extractor
    // 0 = SUPERPOINT, 1 = DISK
    int extractor_type = 0; 
    
    std::shared_ptr<Extractor_DPL> extractor = std::make_shared<Extractor_DPL>();
    try {
        cout << "Loading ONNX model from: " << onnx_path << endl;
        extractor->initialize(onnx_path, extractor_type);
    } catch (const std::exception& e) {
        cerr << "Failed to initialize extractor: " << e.what() << endl;
        return 1;
    }

    vector<cv::Mat> all_descriptors;
    int count = 0;
    int total_features = 0;

    for(const auto &path : images) {
        cv::Mat im = cv::imread(path);
        if(im.empty()) continue;

        // Preprocess following the same logic as in FeatureTrackerDPL::Extractor_PreProcess
        float img_scale = 1.0;
        
        // Extractor_DPL::pre_process returns a normalized image ready for inference
        cv::Mat resultImage = extractor->pre_process(im, img_scale);
        
        // Run inference
        auto result = extractor->extract_featurepoints(resultImage); 
        // result.second is float* of descriptors.
        
        int n_points = result.first.size();
        if (n_points == 0) {
            // cout << "No features found in " << path << endl;
            continue;
        }

        // Convert float* to cv::Mat
        int desc_size = 256; // SuperPoint size
        cv::Mat desc_mat(n_points, desc_size, CV_32F);
        
        for(int i=0; i<n_points; i++) {
            for(int j=0; j<desc_size; j++) {
                desc_mat.at<float>(i, j) = result.second[i*desc_size + j];
            }
            // SuperPoint descriptors should be L2-normalized for DBoW3
            cv::normalize(desc_mat.row(i), desc_mat.row(i), 0, 1, cv::NORM_L2);
        }

        all_descriptors.push_back(desc_mat);
        total_features += n_points;
        delete[] result.second;
        
        count++;
        if(count % 50 == 0) cout << "Processed " << count << "/" << images.size() << " images. Total features: " << total_features << endl;
    }

    if(all_descriptors.empty()) {
        cerr << "No descriptors extracted from any image." << endl;
        return 1;
    }

    cout << "Extracted " << total_features << " features from " << count << " images." << endl;
    cout << "Training vocabulary..." << endl;
    
    // Default params: k=10, L=6
    Vocabulary vocab(10, 6);
    vocab.create(all_descriptors);

    cout << "Vocabulary info: " << vocab << endl;

    string output_file = "superpoint1.yml.gz";
    cout << "Saving vocabulary to " << output_file << "..." << endl;
    vocab.save(output_file);
    
    cout << "Done. Move " << output_file << " to the desired location." << endl;

    return 0;
}
