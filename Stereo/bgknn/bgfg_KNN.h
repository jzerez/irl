#include <opencv2/core/core.hpp>
#include <opencv2/video/background_segm.hpp>

namespace cv{
	class CV_EXPORTS_W BackgroundSubtractorKNN : public BackgroundSubtractor
	{
		protected:
			Size frameSize;
			int frameType;
			int nframes;
			/////////////////////////
			//very important parameters - things you will change
			////////////////////////
			int history;
			//alpha=1/history - speed of update - if the time interval you want to average over is T
			//set alpha=1/history. It is also usefull at start to make T slowly increase
			//from 1 until the desired T
			float fTb;
			//Tb - threshold on the squared distance from the sample used to decide if it is well described
			//by the background model or not. A typical value could be 2 sigma
			//and that is Tb=2*2*10*10 =400; where we take typical pixel level sigma=10

			/////////////////////////
			//less important parameters - things you might change but be carefull
			////////////////////////
			int nN;//totlal number of samples
			int nkNN;//number on NN for detcting background - default K=[0.1*nN]

			//shadow detection parameters
			bool bShadowDetection;//default 1 - do shadow detection
			unsigned char nShadowDetection;//do shadow detection - insert this value as the detection result - 127 default value
			float fTau;
			// Tau - shadow threshold. The shadow is detected if the pixel is darker
			//version of the background. Tau is a threshold on how much darker the shadow can be.
			//Tau= 0.5 means that if pixel is more than 2 times darker then it is not shadow
			//See: Prati,Mikic,Trivedi,Cucchiarra,"Detecting Moving Shadows...",IEEE PAMI,2003.

			//model data
			int nLongCounter;//circular counter
			int nMidCounter;
			int nShortCounter;
			Mat bgmodel; // model data pixel values
			Mat aModelIndexShort;// index into the models
			Mat aModelIndexMid;
			Mat aModelIndexLong;
			Mat nNextShortUpdate;//random update points per model
			Mat nNextMidUpdate;
			Mat nNextLongUpdate;

			String name_;
		public:
			BackgroundSubtractorKNN();
			BackgroundSubtractorKNN(int _history,  float _dist2Threshold, bool _bShadowDetection=true);

			~BackgroundSubtractorKNN();
			/** @brief Returns the number of last frames that affect the background model
			*/

			void apply(InputArray image, OutputArray fgmask, double learningRate=-1);
			virtual void getBackgroundImage(OutputArray backgroundImage) const;
			virtual void initialize(Size _frameSize, int _frameType);
			virtual void read(const FileNode& fs);
			virtual void write(FileStorage& fs) const;


			CV_WRAP virtual int getHistory() const;
			/** @brief Sets the number of last frames that affect the background model
			*/
			CV_WRAP virtual void setHistory(int history);

			/** @brief Returns the number of data samples in the background model
			*/
			CV_WRAP virtual int getNSamples() const;
			/** @brief Sets the number of data samples in the background model.
			  The model needs to be reinitalized to reserve memory.
			  */
			CV_WRAP virtual void setNSamples(int _nN);//needs reinitialization!

			/** @brief Returns the threshold on the squared distance between the pixel and the sample
			  The threshold on the squared distance between the pixel and the sample to decide whether a pixel is
			  close to a data sample.
			  */
			CV_WRAP virtual double getDist2Threshold() const;
			/** @brief Sets the threshold on the squared distance
			*/
			CV_WRAP virtual void setDist2Threshold(double _dist2Threshold);

			/** @brief Returns the number of neighbours, the k in the kNN.
			  K is the number of samples that need to be within dist2Threshold in order to decide that that
			  pixel is matching the kNN background model.
			  */
			CV_WRAP virtual int getkNNSamples() const;
			/** @brief Sets the k in the kNN. How many nearest neigbours need to match.
			*/
			CV_WRAP virtual void setkNNSamples(int _nkNN);

			/** @brief Returns the shadow detection flag
			  If true, the algorithm detects shadows and marks them. See createBackgroundSubtractorKNN for
			  details.
			  */
			CV_WRAP virtual bool getDetectShadows() const;
			/** @brief Enables or disables shadow detection
			*/
			CV_WRAP virtual void setDetectShadows(bool detectShadows);

			/** @brief Returns the shadow value
			  Shadow value is the value used to mark shadows in the foreground mask. Default value is 127. Value 0
			  in the mask always means background, 255 means foreground.
			  */
			CV_WRAP virtual int getShadowValue() const;
			/** @brief Sets the shadow value
			*/
			CV_WRAP virtual void setShadowValue(int value);

			/** @brief Returns the shadow threshold
			  A shadow is detected if pixel is a darker version of the background. The shadow threshold (Tau in
			  the paper) is a threshold defining how much darker the shadow can be. Tau= 0.5 means that if a pixel
			  is more than twice darker then it is not shadow. See Prati, Mikic, Trivedi and Cucchiarra,
			 *Detecting Moving Shadows...*, IEEE PAMI,2003.
			 */
			CV_WRAP virtual double getShadowThreshold() const;
			/** @brief Sets the shadow threshold
			*/
			CV_WRAP virtual void setShadowThreshold(double threshold);
	};

	/** @brief Creates KNN Background Subtractor
	  @param history Length of the history.
	  @param dist2Threshold Threshold on the squared distance between the pixel and the sample to decide
	  whether a pixel is close to that sample. This parameter does not affect the background update.
	  @param detectShadows If true, the algorithm will detect shadows and mark them. It decreases the
	  speed a bit, so if you do not need this feature, set the parameter to false.
	  */

	CV_EXPORTS_W Ptr<BackgroundSubtractorKNN>
		createBackgroundSubtractorKNN(int history=500, double dist2Threshold=400.0,
				bool detectShadows=true);
}
