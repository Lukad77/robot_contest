#include <algorithm>
#include <chrono>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <errno.h>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "../include/detection.hpp"
#include "../include/tools.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace
{
struct CheckConfig
{
    string modelPath;
    string source;
    string saveDir;
    float score = 0.4f;
    int frames = 30;
    int minHits = 3;
    bool show = true;
    bool draw = true;
    bool printAll = false;
    int roiX = 0;
    int roiY = 0;
    int roiW = COLSIMAGE;
    int roiH = ROWSIMAGE;
};

bool fileExists(const string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool dirExists(const string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool isImageFile(const string &path)
{
    string lower = path;
    transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return lower.size() >= 4 &&
           (lower.find(".jpg") != string::npos ||
            lower.find(".jpeg") != string::npos ||
            lower.find(".png") != string::npos ||
            lower.find(".bmp") != string::npos);
}

vector<string> listImages(const string &dir)
{
    vector<string> files;
    DIR *dp = opendir(dir.c_str());
    if (!dp)
        return files;

    while (dirent *entry = readdir(dp))
    {
        string name = entry->d_name;
        if (name == "." || name == "..")
            continue;

        string path = dir + "/" + name;
        if (fileExists(path) && isImageFile(path))
            files.push_back(path);
    }
    closedir(dp);
    sort(files.begin(), files.end());
    return files;
}

bool makeDirIfNeeded(const string &dir)
{
    if (dir.empty() || dirExists(dir))
        return true;
    return mkdir(dir.c_str(), 0755) == 0 || errno == EEXIST;
}

vector<string> readLabels(const string &modelPath)
{
    vector<string> labels;
    ifstream file(modelPath + "/label_list.txt");
    string line;
    while (getline(file, line))
    {
        if (!line.empty())
            labels.push_back(line);
    }
    return labels;
}

void readParamOrArg(int argc, char **argv, CheckConfig &cfg)
{
    ros::NodeHandle privateNh("~");
    privateNh.param<string>("model_path", cfg.modelPath, cfg.modelPath);
    privateNh.param<string>("source", cfg.source, cfg.source);
    privateNh.param<string>("save_dir", cfg.saveDir, cfg.saveDir);
    privateNh.param<float>("score", cfg.score, cfg.score);
    privateNh.param<int>("frames", cfg.frames, cfg.frames);
    privateNh.param<int>("min_hits", cfg.minHits, cfg.minHits);
    privateNh.param<bool>("show", cfg.show, cfg.show);
    privateNh.param<bool>("draw", cfg.draw, cfg.draw);
    privateNh.param<bool>("print_all", cfg.printAll, cfg.printAll);
    privateNh.param<int>("roi_x", cfg.roiX, cfg.roiX);
    privateNh.param<int>("roi_y", cfg.roiY, cfg.roiY);
    privateNh.param<int>("roi_w", cfg.roiW, cfg.roiW);
    privateNh.param<int>("roi_h", cfg.roiH, cfg.roiH);

    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];
        auto nextValue = [&](string &out) {
            if (i + 1 < argc)
                out = argv[++i];
        };

        if (arg == "--model")
            nextValue(cfg.modelPath);
        else if (arg == "--source")
            nextValue(cfg.source);
        else if (arg == "--save")
            nextValue(cfg.saveDir);
        else if (arg == "--score" && i + 1 < argc)
            cfg.score = stof(argv[++i]);
        else if (arg == "--frames" && i + 1 < argc)
            cfg.frames = stoi(argv[++i]);
        else if (arg == "--min-hits" && i + 1 < argc)
            cfg.minHits = stoi(argv[++i]);
        else if (arg == "--roi-x" && i + 1 < argc)
            cfg.roiX = stoi(argv[++i]);
        else if (arg == "--roi-y" && i + 1 < argc)
            cfg.roiY = stoi(argv[++i]);
        else if (arg == "--roi-w" && i + 1 < argc)
            cfg.roiW = stoi(argv[++i]);
        else if (arg == "--roi-h" && i + 1 < argc)
            cfg.roiH = stoi(argv[++i]);
        else if (arg == "--no-show")
            cfg.show = false;
        else if (arg == "--print-all")
            cfg.printAll = true;
    }
}

Rect toRect(const PredictResult &result, const Size &size)
{
    int x = max(0, result.x);
    int y = max(0, result.y);
    int right = min(size.width - 1, result.x + result.width);
    int bottom = min(size.height - 1, result.y + result.height);
    return Rect(Point(x, y), Point(max(x + 1, right), max(y + 1, bottom)));
}

Point centerOf(const PredictResult &result)
{
    return Point(result.x + result.width / 2, result.y + result.height / 2);
}

bool isPartLabel(const string &label)
{
    return label == LABEL_AI_NUT || label == LABEL_AI_SCREW ||
           label == LABEL_AI_PCB || label == LABEL_AI_BLOCK ||
           label == LABEL_AI_TAPE;
}

Rect normalizeRoi(const CheckConfig &cfg, const Size &size)
{
    int x = max(0, min(cfg.roiX, size.width - 1));
    int y = max(0, min(cfg.roiY, size.height - 1));
    int w = max(1, min(cfg.roiW, size.width - x));
    int h = max(1, min(cfg.roiH, size.height - y));
    return Rect(x, y, w, h);
}

PredictResult selectOrderBox(const vector<PredictResult> &results)
{
    PredictResult order;
    order.score = 0.0f;
    order.x = order.y = order.width = order.height = 0;

    for (const auto &result : results)
    {
        if (result.label != LABEL_AI_ORDER)
            continue;

        int centerY = result.y + result.height / 2;
        int selectedCenterY = order.y + order.height / 2;
        if (order.label != LABEL_AI_ORDER || centerY >= selectedCenterY)
            order = result;
    }
    return order;
}

vector<PredictResult> partsInOrderOrRoi(const vector<PredictResult> &results, const PredictResult &order, const Rect &roi)
{
    vector<PredictResult> parts;
    for (const auto &result : results)
    {
        if (!isPartLabel(result.label))
            continue;

        Point center = centerOf(result);
        bool insideOrder = order.label == LABEL_AI_ORDER &&
                           center.x >= order.x && center.x <= order.x + order.width &&
                           center.y >= order.y && center.y <= order.y + order.height;
        bool insideRoi = roi.contains(center);
        if (insideOrder || (order.label != LABEL_AI_ORDER && insideRoi))
            parts.push_back(result);
    }

    sort(parts.begin(), parts.end(), [](const PredictResult &a, const PredictResult &b) {
        return centerOf(a).y < centerOf(b).y;
    });
    return parts;
}

void drawOrderParts(Mat &img, const PredictResult &order, const vector<PredictResult> &parts, const Rect &roi)
{
    rectangle(img, roi, Scalar(255, 0, 255), 1);
    if (order.label == LABEL_AI_ORDER)
        rectangle(img, toRect(order, img.size()), Scalar(0, 255, 255), 2);

    for (const auto &part : parts)
    {
        Rect rect = toRect(part, img.size());
        rectangle(img, rect, Scalar(0, 255, 0), 2);
        string text = part.label + " " + to_string(part.score).substr(0, 4);
        putText(img, text, Point(rect.x, max(12, rect.y - 4)), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(0, 0, 255), 1);
    }
}

void printFrameSummary(int frameIndex, const vector<PredictResult> &allResults, const vector<PredictResult> &parts)
{
    cout << "Frame " << frameIndex << " order parts:";
    if (parts.empty())
        cout << " none";
    for (const auto &part : parts)
        cout << " " << part.label << "(" << fixed << setprecision(2) << part.score << ")";
    cout << endl;

    if (!allResults.empty())
    {
        cout << "  all detections:";
        for (const auto &result : allResults)
            cout << " " << result.label << "(" << fixed << setprecision(2) << result.score << ")";
        cout << endl;
    }
}

string joinPartLabels(const vector<PredictResult> &parts)
{
    if (parts.empty())
        return "none";

    string value;
    for (size_t i = 0; i < parts.size(); ++i)
    {
        if (i > 0)
            value += " ";
        value += parts[i].label;
    }
    return value;
}

void printUsage()
{
    cout << "Usage:\n"
         << "  rosrun sebot_factory sebot_model_check --model <model_dir> [--source <camera|image|dir|video>] [--frames 30] [--score 0.4]\n"
         << "Examples:\n"
         << "  rosrun sebot_factory sebot_model_check --model /home/sebot/new_model --source camera --frames 60\n"
         << "  rosrun sebot_factory sebot_model_check --model /home/sebot/new_model --source /home/sebot/order_images --no-show\n"
         << "  rosrun sebot_factory sebot_model_check --model /home/sebot/new_model --source /dev/video0 --roi-x 20 --roi-y 20 --roi-w 280 --roi-h 210\n";
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sebot_model_check");

    CheckConfig cfg;
    string pathPkg = ros::package::getPath("sebot_factory");
    cfg.modelPath = pathPkg + "/res/model";
    readParamOrArg(argc, argv, cfg);

    if (cfg.modelPath.empty() || !dirExists(cfg.modelPath))
    {
        cerr << "Model path is invalid: " << cfg.modelPath << endl;
        printUsage();
        return 1;
    }

    vector<string> labels = readLabels(cfg.modelPath);
    cout << "Model path: " << cfg.modelPath << endl;
    cout << "Config: source=" << (cfg.source.empty() ? "camera" : cfg.source)
         << " frames=" << cfg.frames
         << " score=" << cfg.score
         << " min_hits=" << cfg.minHits
         << " roi=(" << cfg.roiX << "," << cfg.roiY << "," << cfg.roiW << "," << cfg.roiH << ")"
         << " show=" << (cfg.show ? "true" : "false")
         << " print_all=" << (cfg.printAll ? "true" : "false") << endl;
    cout << "Labels:";
    for (const auto &label : labels)
        cout << " " << label;
    cout << endl;

    set<string> labelSet(labels.begin(), labels.end());
    vector<string> requiredLabels;
    requiredLabels.push_back(LABEL_AI_ORDER);
    requiredLabels.push_back(LABEL_AI_NUT);
    requiredLabels.push_back(LABEL_AI_SCREW);
    requiredLabels.push_back(LABEL_AI_PCB);
    requiredLabels.push_back(LABEL_AI_BLOCK);
    requiredLabels.push_back(LABEL_AI_TAPE);
    for (const string &required : requiredLabels)
    {
        if (labelSet.find(required) == labelSet.end())
            cout << "Warning: label_list.txt does not contain '" << required << "'" << endl;
    }

    shared_ptr<Detection> detection = make_shared<Detection>(cfg.modelPath);
    detection->score = cfg.score;
    if (!cfg.saveDir.empty() && !makeDirIfNeeded(cfg.saveDir))
        cerr << "Warning: cannot create save_dir: " << cfg.saveDir << endl;

    vector<string> imageFiles;
    VideoCapture capture;
    bool useImages = false;

    if (cfg.source.empty() || cfg.source == "camera")
    {
        string device;
        if (!getVideoDevice(VideoIndex::ASTRA_RGB, device))
            device = "/dev/deepCamera";
        capture.open(device, CAP_V4L2);
        cout << "Source: " << device << endl;
    }
    else if (dirExists(cfg.source))
    {
        imageFiles = listImages(cfg.source);
        useImages = true;
        cfg.frames = imageFiles.size();
        cout << "Source image directory: " << cfg.source << " (" << imageFiles.size() << " images)" << endl;
    }
    else if (fileExists(cfg.source) && isImageFile(cfg.source))
    {
        imageFiles.push_back(cfg.source);
        useImages = true;
        cfg.frames = 1;
        cout << "Source image: " << cfg.source << endl;
    }
    else
    {
        capture.open(cfg.source, CAP_V4L2);
        cout << "Source video/device: " << cfg.source << endl;
    }

    if (!useImages)
    {
        if (!capture.isOpened())
        {
            cerr << "Cannot open source: " << cfg.source << endl;
            return 1;
        }
        capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);
        capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
        capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }

    map<string, int> hitCount;
    map<string, float> bestScore;
    map<string, int> sequenceCount;
    vector<PredictResult> maxParts;
    int framesWithOrder = 0;
    int validFrames = 0;
    int totalParts = 0;
    long totalMs = 0;

    int frameIndex = 0;
    int failedReads = 0;
    while (frameIndex < cfg.frames)
    {
        Mat img;
        if (useImages)
        {
            img = imread(imageFiles[frameIndex]);
            if (img.empty())
            {
                frameIndex++;
                continue;
            }
            resize(img, img, Size(COLSIMAGE, ROWSIMAGE));
        }
        else
        {
            if (!capture.read(img))
            {
                failedReads++;
                if (failedReads % 20 == 0)
                    cerr << "Camera read failed " << failedReads << " times, still waiting..." << endl;
                if (failedReads > 200)
                {
                    cerr << "Cannot read valid frames from camera source." << endl;
                    break;
                }
                ros::Duration(0.05).sleep();
                continue;
            }
        }
        failedReads = 0;

        frameIndex++;
        int currentFrame = frameIndex;
        auto start = chrono::steady_clock::now();
        detection->inference(img);
        auto end = chrono::steady_clock::now();
        long frameMs = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        totalMs += frameMs;

        PredictResult order = selectOrderBox(detection->results);
        Rect roi = normalizeRoi(cfg, img.size());
        vector<PredictResult> parts = partsInOrderOrRoi(detection->results, order, roi);
        if (order.label == LABEL_AI_ORDER)
            framesWithOrder++;
        validFrames++;
        totalParts += parts.size();
        sequenceCount[joinPartLabels(parts)]++;
        if (parts.size() > maxParts.size())
            maxParts = parts;

        for (const auto &part : parts)
        {
            hitCount[part.label]++;
            bestScore[part.label] = max(bestScore[part.label], part.score);
        }

        if (cfg.printAll || !parts.empty())
            printFrameSummary(currentFrame, detection->results, parts);

        if (cfg.draw || cfg.show || !cfg.saveDir.empty())
        {
            Mat vis = img.clone();
            drawOrderParts(vis, order, parts, roi);
            string fpsText = "infer " + to_string(frameMs) + "ms";
            putText(vis, fpsText, Point(6, 18), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

            if (!cfg.saveDir.empty())
            {
                stringstream ss;
                ss << cfg.saveDir << "/model_check_" << setw(4) << setfill('0') << currentFrame << ".jpg";
                imwrite(ss.str(), vis);
            }

            if (cfg.show)
            {
                imshow("sebot_model_check", vis);
                if (waitKey(useImages ? 300 : 1) == 27)
                    break;
            }
        }
    }

    cout << "\n===== Model Check Summary =====" << endl;
    cout << "Frames: " << validFrames << endl;
    cout << "Frames with order board: " << framesWithOrder << endl;
    if (validFrames > 0)
    {
        cout << "Average inference: " << fixed << setprecision(2) << (double)totalMs / validFrames << " ms" << endl;
        cout << "Average order parts/frame: " << fixed << setprecision(2) << (double)totalParts / validFrames << endl;
    }

    cout << "Max parts in one frame: " << maxParts.size() << " [" << joinPartLabels(maxParts) << "]" << endl;

    vector<pair<string, int>> sequences(sequenceCount.begin(), sequenceCount.end());
    sort(sequences.begin(), sequences.end(), [](const pair<string, int> &a, const pair<string, int> &b) {
        return a.second > b.second;
    });
    cout << "Most common order sequences:" << endl;
    for (size_t i = 0; i < sequences.size() && i < 5; ++i)
        cout << "  " << sequences[i].first << ": " << sequences[i].second << " frames" << endl;

    cout << "Part hits inside order board:" << endl;
    vector<pair<string, int>> hits(hitCount.begin(), hitCount.end());
    sort(hits.begin(), hits.end(), [](const pair<string, int> &a, const pair<string, int> &b) {
        return a.second > b.second;
    });

    vector<string> accepted;
    for (const auto &hit : hits)
    {
        cout << "  " << hit.first << ": " << hit.second << " hits, best score "
             << fixed << setprecision(2) << bestScore[hit.first];
        if (hit.second >= cfg.minHits)
        {
            accepted.push_back(hit.first);
            cout << "  ACCEPT";
        }
        cout << endl;
    }

    cout << "Accepted order by min_hits=" << cfg.minHits << ":";
    if (accepted.empty())
        cout << " none";
    for (const auto &label : accepted)
        cout << " " << label;
    cout << endl;
    cout << "================================" << endl;

    if (!useImages)
        capture.release();
    return 0;
}
