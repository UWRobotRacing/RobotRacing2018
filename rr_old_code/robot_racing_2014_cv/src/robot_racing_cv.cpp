#include "robot_racing_cv.hpp"

bool compare_blobs(const point_vector_i A, const point_vector_i B)
{
  return (A.size() < B.size());
}

bool compare_points (const cv::Point2i A, const cv::Point2i B)
{
  if (A.x != B.x)
  {
    return (A.x < B.x);
  }
  else
  {
    return (A.y < B.y);
  }
}

//Searches blobs in descending order (size) for a blob that is roughly circular
void findCircleBlob(const std::vector < point_vector_i > blobs, point_vector_i &circle_blob, const double max_eccentricity)
{ 
  //Iterate through blobs in descending order until blob satisfying the circle requirement is found
  circle_blob.clear();
  cv::Moments blob_moments;
  Eigen::Matrix2d cov_matrix;
  Eigen::Vector2d cov_eigenvalues;
  double min_eigenvalue;
  double max_eigenvalue;
  double eccentricity; 

  for(std::vector < point_vector_i >::const_reverse_iterator rit = blobs.rbegin(); rit != blobs.rend(); ++rit)
  {
    blob_moments = cv::moments(*rit, false);
    //Using eccentricity formula from example on Wikipedia under "Image moment"
    //Compute cov matrix
    double mu_prime_2_0, mu_prime_0_2, mu_prime_1_1;
    mu_prime_2_0 = blob_moments.mu20/blob_moments.m00;
    mu_prime_0_2 = blob_moments.mu02/blob_moments.m00;
    mu_prime_1_1 = blob_moments.mu11/blob_moments.m00;    
    cov_matrix.row(0) << mu_prime_2_0, mu_prime_1_1;
    cov_matrix.row(1) << mu_prime_1_1, mu_prime_0_2;

    //Now need to find the eigenvalues of the cov matrix:
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov_matrix);
    if (eigensolver.info() != Eigen::Success)
    {
      continue;
      ROS_ERROR("Eigenvalue computation failed to converge. This should never genuinely happen");
      abort();
    }
    cov_eigenvalues = eigensolver.eigenvalues();

    if(fabs(cov_eigenvalues.x()) > fabs(cov_eigenvalues.y()))
    {//X is bigger
      max_eigenvalue = fabs(cov_eigenvalues.x());
      min_eigenvalue = fabs(cov_eigenvalues.y());
    }
    else
    {
      max_eigenvalue = fabs(cov_eigenvalues.y());
      min_eigenvalue = fabs(cov_eigenvalues.x());
    }

    eccentricity = 1.0 - min_eigenvalue/max_eigenvalue;

    //Check eccentricity
    if ( (eccentricity < max_eccentricity) && blob_moments.m00 > 150)
    {
      //current blob is close enough to a circle!
      circle_blob = *rit;
      return;
    }
  }
}

void findBlobs(const cv::Mat &binary, std::vector < point_vector_i > &blobs)
{
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32SC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

            point_vector_i blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            if(blob.size() > 20)
            {
              blobs.push_back(blob);
              label_count++;              
            }

        }
    }
    std::sort(blobs.begin(), blobs.end(), compare_blobs);
}

void removeScatteredSubvector(point_vector_i &supervector, point_vector_i remove_me)
{
  //copy vector to be removed
  point_vector_i subvector = remove_me;
  //temporary output vector
  point_vector_i t_vector;
  t_vector.clear();

  //Ensure both vectors are sorted
  std::sort(supervector.begin(), supervector.end(), compare_points);
  std::sort(subvector.begin(), subvector.end(), compare_points);

  point_vector_i::iterator iterator_supervector = supervector.begin();
  point_vector_i::iterator iterator_subvector = subvector.begin();

  bool is_point_same;
  bool is_subvector_finished = false;

  //Iterate over entire supervector
  for(iterator_supervector = supervector.begin(); iterator_supervector != supervector.end(); ++iterator_supervector)
  {
    if(!is_subvector_finished)
    {//vector hasn't been excluded yet

      //Compare current iterator points.
      is_point_same = ((iterator_subvector->x == iterator_supervector->x) && (iterator_subvector->y == iterator_supervector->y));
      if(is_point_same)
      {//Point is the same. Iterate subvector and do not copy out
        iterator_subvector++;
        is_subvector_finished = (iterator_subvector == subvector.end());
      }
      else
      {//Points are different. Copy out supervector point
        t_vector.push_back(*iterator_supervector);
      }
    }
    else
    {//vector has been removed...keep copying out
      t_vector.push_back(*iterator_supervector);
    }
  }//For: next supervector point

  //New supervector is t_vector
  supervector = t_vector;
}

NeuralNetLaneClassifier::NeuralNetLaneClassifier(std::string &net_tx_1_path, 
                                                 std::string &net_tx_2_path,
                                                 cv::Size &net_tx_1_size,
                                                 cv::Size &net_tx_2_size,
                                                 cv::Size &classifier_window_size)
{
  //Initialize arrays of doubles
  int net_tx_1_numel = net_tx_1_size.height * net_tx_1_size.width;
  int net_tx_2_numel = net_tx_2_size.height * net_tx_2_size.width;
  double * net_tx_1_array = new double[net_tx_1_numel];
  double * net_tx_2_array = new double[net_tx_2_numel];
  
  //Convert the array into the net_tx_1 cv::Mat object
  cv::Mat temp_net_tx_1 = cv::Mat::zeros(net_tx_1_size, cv::DataType<double>::type);
  //Convert the array into the net_tx_2 cv::Mat object
  cv::Mat temp_net_tx_2 = cv::Mat::zeros(net_tx_2_size, cv::DataType<double>::type);

  //////////
  //net_tx_1
  //////////
  //Open binary file for net_tx_1
  FILE * file_tx_1;
  file_tx_1 = fopen(net_tx_1_path.c_str(), "rb");
  //Check that file access was successful
  if (file_tx_1==NULL)
  {
    std::cout << "file read error! net_tx_1" << std::endl;
  }
  //Read the array for net_tx_1
  size_t result_1 = fread(net_tx_1_array,sizeof(double),net_tx_1_numel, file_tx_1);
  //Verify read result
  fclose(file_tx_1);
  assert(result_1 == net_tx_1_numel);

  for(int i_col = 0; i_col < net_tx_1_size.width; i_col++)
  {
    for(int i_row = 0; i_row < net_tx_1_size.height; i_row++)
      temp_net_tx_1.at<double>(i_row,i_col) = net_tx_1_array[i_col * net_tx_1_size.height + i_row];
  }
  temp_net_tx_1.copyTo(net_tx_1);
  delete[] net_tx_1_array;

  //////////
  //net_tx_2
  //////////
  //Open binary file for net_tx_2
  FILE * file_tx_2;
  file_tx_2 = fopen(net_tx_2_path.c_str(), "rb");
  //Check that file access was successful
  if (file_tx_1==NULL)
  {
    std::cout << "file read error! net_tx_1" << std::endl;
  }  
  //Read the array for net_tx_1
  size_t result_2 = fread(net_tx_2_array,sizeof(double),net_tx_2_numel, file_tx_2);
  //Verify read result
  fclose(file_tx_2);
  assert(result_2 == net_tx_2_numel);

  for(int i_col = 0; i_col < net_tx_2_size.width; i_col++)
  {
    for(int i_row = 0; i_row < net_tx_2_size.height; i_row++)
      temp_net_tx_2.at<double>(i_row,i_col) = net_tx_2_array[i_col * net_tx_2_size.height + i_row];
  }
  temp_net_tx_2.copyTo(net_tx_2);
  delete[] net_tx_2_array;

  ///////////////////
  //classifier_window
  ///////////////////
  classifier_window = classifier_window_size;
}

cv::Mat NeuralNetLaneClassifier::getMat_net_tx_1()
{
  return net_tx_1;
}

cv::Mat NeuralNetLaneClassifier::getMat_net_tx_2()
{
  return net_tx_2;
}

void NeuralNetLaneClassifier::classify_image(const cv::Mat &input_image, const cv::Mat &mask, cv::Mat &output_image)
{

  //Check that the mask and input image have the same size
  assert(input_image.size() == mask.size());

  cv::Mat clone_input = input_image.clone();

  //Initialize output matrix
  cv::Mat classified_image = cv::Mat::zeros(clone_input.size(),CV_8U);

  //Create a dupe mask matrix
  cv::Mat clone_mask = mask.clone();

  //Initialize patch vector
  cv::Size vector_size = cv::Size(1,classifier_window.width * classifier_window.height);
  cv::Mat patch_vector = cv::Mat::zeros(vector_size,CV_8U);

  //Initialize feature vector
  cv::Mat feature_vector = cv::Mat::zeros(vector_size,CV_64F);

  //Initialize patch submatrix
  cv::Mat image_patch;

  //Patch index bounds
  int x_l_bound, x_u_bound, y_l_bound, y_u_bound;

  //Flipped patch index bounds
  int flip_x_l_bound, flip_x_u_bound, flip_y_l_bound, flip_y_u_bound;  

  //Pointers for extracting patch vector because openCV's functions are awful for this (if matlab format needed)
  uchar *imagerowPtr;
  uchar *patchvecPtr;

  //Pointers for output matrix
  uchar *outputrowPtr;

  //Pointer to mask image
  uchar *maskrowPtr;

  bool pixel_class;
  bool flip_pixel_class;

  double t = (double)cv::getTickCount();
  //Iterate over all pixels whose neighbourhood is inside the image
  for(int i_row = 0; i_row < clone_input.size().height; i_row++)
  {
    for(int i_col = 0; i_col < clone_input.size().width; i_col++)
    {
      //Check ####ADD in MASK!!!!
      maskrowPtr = clone_mask.ptr<uchar>(i_row); 
      if(maskrowPtr[i_col] == 0)
      {
        continue;
      }

      ///////////////////////////////
      //Original Patch Classification
      ///////////////////////////////
      //Calculate patch pixel limits
      x_l_bound = i_col - (classifier_window.width -1)/2;
      x_u_bound = i_col + (classifier_window.width -1)/2;
      y_l_bound = i_row - (classifier_window.height -1)/2;
      y_u_bound = i_row + (classifier_window.height -1)/2;
      //Check bounds
      if((x_l_bound < 0) ||
          (x_u_bound > clone_input.size().width-1) ||
          (y_l_bound < 0) ||
          (y_u_bound > clone_input.size().height-1))
      {
        pixel_class = false;
      }
      else
      {
        image_patch = clone_input(cv::Range(y_l_bound,y_u_bound+1),cv::Range(x_l_bound,x_u_bound+1));
        for(int i_patch_col = 0; i_patch_col < classifier_window.width; i_patch_col++)
        {
          for(int i_patch_row = 0; i_patch_row < classifier_window.height; i_patch_row++)
          {
            imagerowPtr = image_patch.ptr<uchar>(i_patch_row);
            patchvecPtr = patch_vector.ptr<uchar>(i_patch_col*classifier_window.height+i_patch_row);
            patchvecPtr[0] = imagerowPtr[i_patch_col];
          }
        }
        //Convert patch_vector to feature_vector
        patch_vector.convertTo(feature_vector, CV_64F);
        //Normalize patch vector
        feature_vector= feature_vector*2/255-1;

        //Evaluate pixel patch
        pixel_class = classify_patch(feature_vector);
      }

      //////////////////////////////
      //Flipped Patch Classification
      //////////////////////////////
      //Calculate patch pixel limits
      flip_x_l_bound = i_col - (classifier_window.height -1)/2;
      flip_x_u_bound = i_col + (classifier_window.height -1)/2;
      flip_y_l_bound = i_row - (classifier_window.width -1)/2;
      flip_y_u_bound = i_row + (classifier_window.width -1)/2;
      //Check bounds
      if((x_l_bound < 0) ||
          (x_u_bound > clone_input.size().height-1) ||
          (y_l_bound < 0) ||
          (y_u_bound > clone_input.size().width-1))
      {
        pixel_class = false;
      }
      else
      {
        image_patch = clone_input(cv::Range(y_l_bound,y_u_bound+1),cv::Range(x_l_bound,x_u_bound+1));
        for(int i_patch_col = 0; i_patch_col < classifier_window.height; i_patch_col++)
        {
          for(int i_patch_row = 0; i_patch_row < classifier_window.width; i_patch_row++)
          {
            imagerowPtr = image_patch.ptr<uchar>(i_patch_row);
            patchvecPtr = patch_vector.ptr<uchar>(i_patch_col*classifier_window.width+i_patch_row);
            patchvecPtr[0] = imagerowPtr[i_patch_col];
          }
        }
        //Convert patch_vector to feature_vector
        patch_vector.convertTo(feature_vector, CV_64F);
        //Normalize patch vector
        feature_vector= feature_vector*2/255-1;

        //Evaluate pixel patch
        flip_pixel_class = classify_patch(feature_vector);
      }
      
      if((pixel_class == true) || (flip_pixel_class == true))
      {//Lane Marking!
        //Get pointer to output image at this row
        outputrowPtr = classified_image.ptr<uchar>(i_row);
        //Set row array to 255 at this column
        outputrowPtr[i_col] = 255;        
      }
    }
  }
  classified_image.copyTo(output_image);
  return;
}

bool NeuralNetLaneClassifier::classify_patch(const cv::Mat &input_vector)
{
  cv::Mat aug_vec_1 = input_vector.clone();
  cv::Mat sum_into_hidden;
  cv::Mat hidden_layer_output;
  cv::Mat sum_into_output;
  cv::Mat output_layer;
  double temp_value;
  double lane_class_val;
  double null_class_val;

  //Append 1 for bias term
  aug_vec_1.push_back(1.0);

  //Calculate input to hidden layer
  sum_into_hidden = net_tx_1 * aug_vec_1;

  //Output of hidden layer
  sum_into_hidden.copyTo(hidden_layer_output);
  double *hidden_output_row_ptr;
  for(int i_hidden = 0; i_hidden < sum_into_hidden.total(); i_hidden++)
  {
    hidden_output_row_ptr = hidden_layer_output.ptr<double>(i_hidden);
    temp_value = hidden_output_row_ptr[0]; //Copy value to temp
    hidden_output_row_ptr[0] = 2.0 / (1 + std::exp(-2*temp_value)) - 1;    
  }

  //Append 1 for bias term
  hidden_layer_output.push_back(1.0);

  //Calculate input to output layer
  sum_into_output = net_tx_2 * hidden_layer_output;

  //Output of output layer
  sum_into_output.copyTo(output_layer);
  double *output_layer_row_ptr;
  output_layer_row_ptr = output_layer.ptr<double>(0);
  lane_class_val = output_layer_row_ptr[0];
  output_layer_row_ptr = output_layer.ptr<double>(1);
  null_class_val = output_layer_row_ptr[1];

  if(lane_class_val > null_class_val)
  {
    return true;
  }
  else
  {
    return false;
  }
}