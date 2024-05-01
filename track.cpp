#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

class Track{
    public:
        // Track(string centers_file, string tangent_file, string normals_file) :
        //     track_centers_file(centers_file),
        //     track_tangent_file(tangent_file),
        //     track_normals_file(centers_file),
        // {};

        Track(string a, string b, string c);
        void read_record();

        ArrayX2d track_centers;
        ArrayXd track_tangent;
        ArrayXd track_normals;

        string track_centers_file;
        string track_tangent_file;
        string track_normals_file;

        // must add operator for assignment operator

        vector<double> nearest_trackpoint(ArrayXd& p);

};

Track::Track(string a, string b, string c):
    track_centers_file(a),
    track_tangent_file(b),
    track_normals_file(c)
{
    read_record(); // only track_centers is read as of now
};

void Track::read_record() { 
	fstream fin; 
    fin.open(track_centers_file, ios::in);

    int row_num = 0;
    string line;
    vector<string> row; 

    // Read the file line by line
    while (getline(fin, line)) { 
        row.clear();
        stringstream s(line);
        string word;

        // Split the line into comma-separated values
        while (getline(s, word, ',')) { 
            row.push_back(word);
        }

        // Convert string values to doubles and store them into track_centers
        if (row.size() >= 2) {
            track_centers(row_num, 0) = stod(row[0]);
            track_centers(row_num, 1) = stod(row[1]);
        }

        row_num++;
    } 

    fin.close(); // Don't forget to close the file
} 

vector<double> Track::nearest_trackpoint(ArrayXd& p){
    // int i = ((track_centers - p.replicate(track_centers.rows(), 1)).square().rowwise().sum()).argmin();
    // vector<double> ans;

    // ans.push_back(i);
    // ans.push_back(track_centers(i, 0));
    // ans.push_back(track_centers(i, 1));
    // ans.push_back(track_tangent(i));
    // ans.push_back(track_normals(i));

    // return ans;
}


int main(){
    Track track("track_centers.csv", "b", "c");

    cout << track.track_centers;

    return 0;
}
