#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <pcl/io/vtk_lib_io.h>

using namespace pcl;
using namespace std;

// inputs are text files that need to be converted to pcd files
int main(int argc, char **argv)
{  
    string input_file;
    string current_line;
    char delimit = ',';
    string point;
    stringstream line;
    vector<float> points;

    for(int i = 1; i < argc; i++)
    {
        input_file = argv[i];
        // open the txt file for inputs
        ifstream file(input_file);
        // loop through each line in the txt file
        while(getline(file, current_line))
        {
            line.str(current_line);
            // loop through each point in the line
            while(getline(line, point, delimit))
            {
                // save the points in a vector
                points.push_back(stof(point));
            }
            line.clear();
        }

        int counter = 1;
        stringstream output_stream;
        // remove the txt extension
        input_file.erase(input_file.size() - 1);
        input_file.erase(input_file.size() - 1);
        input_file.erase(input_file.size() - 1);
        // add the pcd extension
        output_stream << input_file << "pcd";
        string output_file = output_stream.str();
        // open the pcd file stream for outputs
        ofstream out(output_file);
        // add the pcd ASCII header
        out << "# .PCD v.7 - Point Cloud Data file\n" << "VERSION .7\n" << "FIELDS x y z\n" << "SIZE 4 4 4\n" << "TYPE F F F\n" << "COUNT 1 1 1\n"
        << "WIDTH 1\n" << "HEIGHT " << points.size()/3 << "\nVIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << points.size()/3 << "\nDATA ascii\n";
        for(int i = 0; i < points.size(); i++) {
            // output the points to the pcd file
            if(counter % 3 != 0)
            out << points[i] << " ";
            else
            out << points[i] << "\n";
            counter++;
        }
        out.close();
        cout << output_file << " was created and saved successfully." << endl;  
    }
}

