/*
	*********************	Hide and Seek   ********************
	Hide a linear frequency-modulated (LFM) pulse of given pulse duration and bandwidth in Gaussian noise.
	Also called a chirp waveform and used in sonar and radar applications.  User enters Where (ms), pulse width (ms),
	bandwidth (Hz), Samples (integer), Sample Rate (Hz), and Signal-to-Noise ratio (SNR in dB).  The location of
	the pulse is shown in red in the plot.  The "Hide" button displays the pulse in Gaussian noise.
	The "Seek" button is the matched filter output. The LFM is a form of pulse compression which increases
	the range resolution.  The noisy signal is cross-correlated with a matched filter, which is a replica of
	the LFM.  This processing is similar to a convolution sum, except the filter is not reversed in time.
	The tradeoff is between the pulse duration which increases the SNR but decreases the range resolution.
	Pulse compression allows you to increase the pulse duration thereby increasing the transmitted energy,
	but the waveform is compresssed in the receiver processing thereby increasing the range resolution.
	So you get increased SNR and range resolution at the same time.

	This program simulates using Direct Memory Access (DMA) with interrupts on a Digital Signal Processor (DSP).
	Multiple goroutines allow generating the signal and filtering the signal concurrently.  Mulitple-
	core processors will optimize this approach.  The signal generator goroutine will produce samples
	of the signal and synchronize with the filter goroutine when done creating a block of the samples.
	These will be placed in one buffer.  The filter routine will process the samples in that buffer.  In
	the meantime, the generator goroutine will start filling another buffer with samples.  It will then
	synchronize with the filter goroutine when the filter is ready to accept the new block.  The cycle repeats
	until all the samples are produced and filtered.  This is a ping-pong technique where the buffers alternate
	between generating and filtering.  Synchronization is done using unbuffered channels which behave as semaphores.
*/

package main

import (
	"bufio"
	"fmt"
	"html/template"
	"log"
	"math"
	"math/rand"
	"net/http"
	"os"
	"path"
	"strconv"
	"sync"
)

const (
	rows                       = 300  // #rows in grid
	columns                    = 300  // #columns in grid
	block                      = 1024 // size of buf1 and buf2, chunks of data to process
	patternhideandseek         = "/hideandseek"
	tmplhideandseek            = "templates/hideandseek.html"
	addr                       = "127.0.0.1:8080" // http server listen address
	xlabels                    = 11               // # labels on x axis
	ylabels                    = 11               // # labels on y axis
	dataDir                    = "data/"          // directory for the signal
	twoPi              float64 = 2.0 * math.Pi
	signal                     = "signal.txt" //  signal
)

// Type to contain all the HTML template actions
type PlotT struct {
	Grid       []string // plotting grid
	Status     string   // status of the plot
	Xlabel     []string // x-axis labels
	Ylabel     []string // y-axis labels
	SampleFreq string   // data sampling rate in Hz
	Samples    string   // complex samples in data file
	SNR        string   // signal-to-noise ratio
	Pulsewidth string   // millisecond
	Bandwidth  string   // Hz
	Where      string   // millisecond
	Location   string   // millisecond
}

// linear frequency modulation waveform properties
type LFM struct {
	Where      float64 // pulse location in ms
	Pulsewidth float64 // pulse width in ms
	Bandwidth  float64 // bandwidth in Hz
	SNR        float64 // signal-to-noise ratio in dB
	CenterFreq float64 // center frequency of pulse in Hz
	Sigma      float64 // standard deviation of Gaussian noise
}

// previous sample block properties used for generating/filtering current block
type FilterState struct {
	lastFiltered    []float64 // last M-1 incomplete filtered samples from previous block
	firstSampleTime float64   // start time of current submit
	lastSampleTime  float64   // end time of currrent submit or block
	lastPulseIndex  int       // end index of the LFM pulse for the block
}

type FilterSignal struct {
	sema1            chan int // semaphores to synchronize access to the ping-pong buffers
	sema2            chan int
	wg               sync.WaitGroup
	buf              [][]float64   // ping-pong buffer
	done             chan struct{} // generator Signal to the filter when all samples generated
	samples          int           // total number of samples per submit
	samplesGenerated int           // number of samples generated so far for this submit
	samplesFiltered  int           // number of samples filtered so far for this submit
	sampleFreq       int           // sample frequency in Hz
	FilterState                    // used by current sample block from previous sample block
	filterCoeff      []float64     // filter coefficients
	Endpoints                      // embedded struct for grid boundaries
	LFM                            // embedded struct for LFM properties
}

// Type to hold the minimum and maximum data values
type Endpoints struct {
	xmin float64
	xmax float64
	ymin float64
	ymax float64
	nmax int // index of the maximum match filter output
}

var (
	hideAndSeekTmpl *template.Template
)

// init parses the HTML template file
func init() {
	hideAndSeekTmpl = template.Must(template.ParseFiles(tmplhideandseek))
}

// createReplica generates the LFM waveform used for match filtering on signal
func (fs *FilterSignal) createReplica() {
	// Create the LFM waveform using A*cos(2*PI*(fc-bw/2)*t + PI*(bw/tau)*t*t)
	// where fc is the center frequency, bw is the bandwidth, tau is the pulse
	// duration in seconds
	delta := 1.0 / float64(fs.sampleFreq)
	A := 1.0
	// Create the LFM pulse and save for match filter using cross-correlation with signal
	for t := 0.0; t < fs.Pulsewidth; t += delta {
		ang := twoPi*(float64(fs.CenterFreq)-float64(fs.Bandwidth)/2.0)*t +
			math.Pi*(float64(fs.Bandwidth)/fs.Pulsewidth)*t*t
		val := A * math.Cos(ang)
		// save the replica for match filter cross-correlation
		fs.filterCoeff = append(fs.filterCoeff, val)
	}
}

// fillBuf populates the buffer with signal samples
func (fs *FilterSignal) fillBuf(n int) int {
	// get last sample time from previous block
	// fill buf n with an LFM waveform with given SNR
	// Sum the LFM and and noise and insert into the buffer

	// Determine how many samples we need to generate
	howMany := block
	toGo := fs.samples - fs.samplesGenerated
	if toGo < block {
		howMany = toGo
	}

	delta := 1.0 / float64(fs.sampleFreq)
	t := fs.lastSampleTime
	// calculate the end of the LFM pulse
	endPulse := fs.Where + fs.Pulsewidth
	j := fs.lastPulseIndex
	lenmf := len(fs.filterCoeff)
	for i := 0; i < howMany; i++ {
		// Insert the LFM pulse at the desired location along with noise
		if t >= fs.Where && t < endPulse && j < lenmf {
			fs.buf[n][i] = fs.filterCoeff[j] + fs.Sigma*rand.NormFloat64()
			j++
			// Insert only the noise
		} else {
			fs.buf[n][i] = fs.Sigma * rand.NormFloat64()
		}
		t += delta
	}

	// Save the next sample time for next block of samples
	// Save the LFM pulse index for the start of the next block of samples
	fs.lastSampleTime = t
	fs.samplesGenerated += howMany
	fs.lastPulseIndex = j
	return howMany
}

// gridFillInterp inserts the data points in the grid and draws a straight line between points
func (fs *FilterSignal) gridFillInterp(plot *PlotT) error {
	var (
		x            float64 = fs.firstSampleTime
		y            float64 = 0.0
		prevX, prevY float64
		err          error
		xscale       float64
		yscale       float64
		input        *bufio.Scanner
		timeStep     float64 = 1.0 / float64(fs.sampleFreq)
		pulseStart   float64 = fs.Where
		pulseEnd     float64 = pulseStart + fs.Pulsewidth
		mflen        int     = len(fs.filterCoeff)
		mfMax        float64 = float64(fs.nmax) * timeStep
		mfStart      float64 = mfMax - 15*timeStep
		mfEnd        float64 = mfMax + 15*timeStep
	)

	// Mark the data x-y coordinate online at the corresponding
	// grid row/column.

	fs.xmin = fs.firstSampleTime
	fs.xmax = fs.lastSampleTime

	// Calculate scale factors for x and y
	xscale = (columns - 1) / (fs.xmax - fs.xmin)
	yscale = (rows - 1) / (fs.ymax - fs.ymin)

	f, err := os.Open(path.Join(dataDir, signal))
	if err != nil {
		fmt.Printf("Error opening %s: %v\n", signal, err.Error())
		return err
	}
	defer f.Close()
	input = bufio.NewScanner(f)

	// if using matched filter skip first matched filter length - 1 samples
	if fs.nmax > 0 {
		for i := 0; i < mflen-1; i++ {
			input.Scan()
		}
	}

	// Get first sample
	input.Scan()
	value := input.Text()

	if y, err = strconv.ParseFloat(value, 64); err != nil {
		fmt.Printf("gridFillInterp first sample string %s conversion to float error: %v\n", value, err)
		return err
	}

	plot.Grid = make([]string, rows*columns)

	// This cell location (row,col) is on the line
	row := int((fs.ymax-y)*yscale + .5)
	col := int((x-fs.xmin)*xscale + .5)
	plot.Grid[row*columns+col] = "online"

	prevX = x
	prevY = y

	// Scale factor to determine the number of interpolation points
	lenEPy := fs.ymax - fs.ymin
	lenEPx := fs.xmax - fs.xmin

	// Continue with the rest of the points in the file
	for input.Scan() {
		x += timeStep
		value = input.Text()
		if y, err = strconv.ParseFloat(value, 64); err != nil {
			fmt.Printf("gridFillInterp the rest of file string %s conversion to float error: %v\n", value, err)
			return err
		}

		// This cell location (row,col) is on the line
		row := int((fs.ymax-y)*yscale + .5)
		col := int((x-fs.xmin)*xscale + .5)
		// if no match filter (hide), mark the pulse in red
		if fs.nmax == 0 {
			if x >= pulseStart && x < pulseEnd {
				plot.Grid[row*columns+col] = "pulse"
			} else {
				plot.Grid[row*columns+col] = "online"
			}
		} else {
			if x >= mfStart && x <= mfEnd {
				plot.Grid[row*columns+col] = "pulse"
			} else {
				plot.Grid[row*columns+col] = "online"
			}
		}

		// Interpolate the points between previous point and current point

		/* lenEdge := math.Sqrt((x-prevX)*(x-prevX) + (y-prevY)*(y-prevY)) */
		lenEdgeX := math.Abs((x - prevX))
		lenEdgeY := math.Abs(y - prevY)
		ncellsX := int(columns * lenEdgeX / lenEPx) // number of points to interpolate in x-dim
		ncellsY := int(rows * lenEdgeY / lenEPy)    // number of points to interpolate in y-dim
		// Choose the biggest
		ncells := ncellsX
		if ncellsY > ncells {
			ncells = ncellsY
		}

		stepX := (x - prevX) / float64(ncells)
		stepY := (y - prevY) / float64(ncells)

		// loop to draw the points
		interpX := prevX
		interpY := prevY
		for i := 0; i < ncells; i++ {
			row := int((fs.ymax-interpY)*yscale + .5)
			col := int((interpX-fs.xmin)*xscale + .5)
			// if no match filter (hide), mark the pulse in red
			if fs.nmax == 0 {
				if x >= pulseStart && x < pulseEnd {
					plot.Grid[row*columns+col] = "pulse"
				} else {
					plot.Grid[row*columns+col] = "online"
				}
			} else {
				if x >= mfStart && x <= mfEnd {
					plot.Grid[row*columns+col] = "pulse"
				} else {
					plot.Grid[row*columns+col] = "online"
				}
			}
			interpX += stepX
			interpY += stepY
		}

		// Update the previous point with the current point
		prevX = x
		prevY = y
	}
	return nil
}

// filterBuf filters the samples with the given matched filter coefficients.
// index is the buffer to use, 1 or 2, nsamples is the number of samples to filter
func (fs *FilterSignal) filterBuf(index int, nsamples int, f *os.File) {
	// have the last M-1 filter outputs from previous block in FilterSignal
	// loop over the samples from the generator and apply the filter coefficients
	// in a convolution sum

	// Incorporate the previous block partially filtered outputs
	m := len(fs.filterCoeff)
	end := m - 1
	if nsamples < m-1 {
		end = nsamples
	}
	for n := 0; n < end; n++ {
		sum := fs.lastFiltered[n]
		for k := 0; k <= n; k++ {
			sum += fs.buf[index][k] * fs.filterCoeff[m-n+k-1]
		}
		// find min/max of the signal as we go
		if sum < fs.ymin {
			fs.ymin = sum
		}
		if sum > fs.ymax {
			fs.ymax = sum
			fs.nmax = n + fs.samplesFiltered - m + 1
		}
		fmt.Fprintf(f, "%f\n", sum)
	}

	// this section of the block have samples for all the coefficients
	if end == m-1 {
		for n := 0; n <= nsamples-m; n++ {
			sum := 0.0
			for k := 0; k < m; k++ {
				sum += fs.buf[index][n+k] * fs.filterCoeff[k]
			}
			// find min/max of the signal as we go
			if sum < fs.ymin {
				fs.ymin = sum
			}
			if sum > fs.ymax {
				fs.ymax = sum
				fs.nmax = n + fs.samplesFiltered
			}
			fmt.Fprintf(f, "%f\n", sum)
		}
	}

	// Generate the partially filtered outputs used in next block
	i := 0
	if nsamples == block {
		for n := block - m + 1; n < block; n++ {
			sum := 0.0
			for k := n; k < block; k++ {
				sum += fs.buf[index][k] * fs.filterCoeff[k-n]
			}
			fs.lastFiltered[i] = sum
			i++
		}
		// save last m-1 partial correlations
	} else {
		// enough samples in this block to correlate
		if nsamples >= m-1 {
			for n := nsamples - m + 1; n < nsamples; n++ {
				sum := 0.0
				for k := n; k < nsamples; k++ {
					sum += fs.buf[index][k] * fs.filterCoeff[k-n]
				}
				fmt.Fprintf(f, "%f\n", sum)
			}
			// partial correlation from this block
		} else {
			for n := 0; n < nsamples; n++ {
				sum := 0.0
				for k := n; k < nsamples; k++ {
					sum += fs.buf[index][k] * fs.filterCoeff[k-n]
				}
				fmt.Fprintf(f, "%f\n", sum)
			}
		}
	}
	fs.samplesFiltered += nsamples
}

// nofilterBuf saves the signal to a file.  It is not modified.
// index is the buffer to use, 1 or 2, nsamples is the number of samples to filter
func (fs *FilterSignal) nofilterBuf(index int, nsamples int, f *os.File) {
	for n := 0; n < nsamples; n++ {
		// find min/max of the signal as we go
		if fs.buf[index][n] < fs.ymin {
			fs.ymin = fs.buf[index][n]
		}
		if fs.buf[index][n] > fs.ymax {
			fs.ymax = fs.buf[index][n]
		}
		fmt.Fprintf(f, "%f\n", fs.buf[index][n])
	}
}

// generate creates the noisy signal, it is the producer or generator
func (fs *FilterSignal) generate(r *http.Request) error {

	// get SNR, pulse duration, bandwidth, where to place the pulse
	temp := r.FormValue("snr")
	if len(temp) == 0 {
		return fmt.Errorf("missing SNR for LFM signal")
	}
	snr, err := strconv.Atoi(temp)
	if err != nil {
		return err
	}

	temp = r.FormValue("bandwidth")
	if len(temp) == 0 {
		return fmt.Errorf("missing bandwidth for LFM signal")
	}
	bandwidth, err := strconv.ParseFloat(temp, 64)
	if err != nil {
		return err
	}

	temp = r.FormValue("pulsewidth")
	if len(temp) == 0 {
		return fmt.Errorf("missing pulsewidth for LFM signal")
	}
	pulsewidth, err := strconv.ParseFloat(temp, 64)
	if err != nil {
		return err
	}

	// Restrict replica size to block samples, convert ms to sec for pulsewidth
	if pulsewidth/1000.0*float64(fs.sampleFreq) >= block {
		return fmt.Errorf("pulsewidth >= number of samples")
	}

	// check pulsewidth for validity, but first convert pulsewidth from ms to sec
	if pulsewidth <= 0 || pulsewidth/1000.0 > float64(fs.samples)/float64(fs.sampleFreq) {
		return fmt.Errorf("pulsewidth %v must be less than the signal duration:  samples/sample frequency", pulsewidth)
	}

	temp = r.FormValue("where")
	if len(temp) == 0 {
		return fmt.Errorf("missing where pulse location for LFM signal")
	}
	where, err := strconv.ParseFloat(temp, 64)
	if err != nil {
		return err
	}

	// check where for validity:  pulse location must be completely inside the signal duration
	if where/1000.0 < 0.0 || where/1000.0 > (float64(fs.samples)/float64(fs.sampleFreq)-pulsewidth/1000.0) {
		return fmt.Errorf("pulse location %v in where must be within the signal", where)
	}

	// check for aliasing
	if fs.CenterFreq-fs.Bandwidth/2 < 0 {
		return fmt.Errorf("aliasing:  CenterFreq-Bandwidth/2 < 0")
	}
	if fs.CenterFreq+fs.Bandwidth/2 > float64(fs.sampleFreq)/2 {
		return fmt.Errorf("aliasing: CenterFreq + Bandwidth/2 > SampleFreq/2")
	}

	// A^2*tau/(sigma*sigma), where A is the peak pulse amplitude, tau is the pulse width in sec,
	// sigma is the noise standard deviation, E = A^2*tau is the signal energy
	A := 1.0
	// convert pulsewidth from ms to sec
	E := A * A * pulsewidth / 1000.0

	// Calculate the noise standard deviation using the SNR and maxampl
	ratio := math.Pow(10.0, float64(snr)/10.0)
	noiseSD := math.Sqrt(E / ratio)

	fs.LFM = LFM{
		Where:      where / 1000.0,      // convert millisec to sec
		Pulsewidth: pulsewidth / 1000.0, // convert millisec to sec
		Bandwidth:  bandwidth,
		SNR:        float64(snr),
		CenterFreq: float64(fs.sampleFreq / 4.0),
		Sigma:      noiseSD,
	}

	// Create the LFM waveform used for matched filtering the signal
	fs.createReplica()

	// increment wg
	fs.wg.Add(1)

	// launch a goroutine to generate samples
	go func() {
		// if all samples generated, signal filter on done semaphore and return
		defer func() {
			fs.done <- struct{}{}
			fs.wg.Done()
		}()

		// loop to generate a block of signal samples
		// signal the filter when done with each block of samples
		// block on a semaphore until filter goroutine is available
		// set the first sample time equal to the previous last sample time
		for {
			n := fs.fillBuf(0)
			fs.sema1 <- n
			if n < block {
				return
			}
			n = fs.fillBuf(1)
			fs.sema2 <- n
			if n < block {
				return
			}
		}
	}()

	return nil
}

// filter processes the noisy signal, it is the consumer
func (fs *FilterSignal) filter(r *http.Request, plot *PlotT) error {
	// increment wg
	fs.wg.Add(1)

	hideORseek := r.FormValue("hide-seek")
	if len(hideORseek) == 0 {
		return fmt.Errorf("missing hide or seek for LFM signal")
	}

	// hide or seek
	if hideORseek == "hide" {
		f2, _ := os.Create(path.Join(dataDir, signal))

		// launch a goroutine to no-filter generator signal
		// select on the generator semaphores and the done channel
		go func() {
			defer f2.Close()
			defer fs.wg.Done()
			for {
				select {
				case n := <-fs.sema1:
					fs.nofilterBuf(0, n, f2)
				case n := <-fs.sema2:
					fs.nofilterBuf(1, n, f2)
				case <-fs.done:
					return
				}
			}
		}()
		// seek
	} else {

		// allocate memory for filtered output from previous submit
		fs.lastFiltered = make([]float64, len(fs.filterCoeff))

		f2, _ := os.Create(path.Join(dataDir, signal))

		// launch a goroutine to filter generator signal
		// select on the generator semaphores and the done channel
		go func() {
			defer f2.Close()
			defer fs.wg.Done()
			for {
				select {
				case n := <-fs.sema1:
					fs.filterBuf(0, n, f2)
				case n := <-fs.sema2:
					fs.filterBuf(1, n, f2)
				case <-fs.done:
					return
				}
			}
		}()
	}

	return nil
}

// label the plot and execute the PlotT on the HTML template
func (fs *FilterSignal) labelExec(w http.ResponseWriter, plot *PlotT) {

	plot.Xlabel = make([]string, xlabels)
	plot.Ylabel = make([]string, ylabels)

	// Construct x-axis labels
	incr := (fs.xmax - fs.xmin) / (xlabels - 1)
	x := fs.xmin
	// First label is empty for alignment purposes
	for i := range plot.Xlabel {
		plot.Xlabel[i] = fmt.Sprintf("%.2f", x)
		x += incr
	}

	// Construct the y-axis labels
	incr = (fs.ymax - fs.ymin) / (ylabels - 1)
	y := fs.ymin
	for i := range plot.Ylabel {
		plot.Ylabel[i] = fmt.Sprintf("%.2f", y)
		y += incr
	}

	// Fill in the form fields
	plot.Samples = strconv.Itoa(fs.samples)
	plot.SampleFreq = strconv.Itoa(fs.sampleFreq)
	plot.SNR = fmt.Sprintf("%.0f", fs.SNR)
	plot.Where = fmt.Sprintf("%.0f", fs.Where*1000.0)           // change to ms from sec
	plot.Pulsewidth = fmt.Sprintf("%.0f", fs.Pulsewidth*1000.0) // change to ms from sec
	plot.Bandwidth = fmt.Sprintf("%.0f", fs.Bandwidth)
	location := ""
	success := "LFM waveform without match filtering"
	// only show location if "Seek" chosen, choose different success status
	if fs.nmax > 0 {
		location = fmt.Sprintf("%.0f", float64(fs.nmax)/float64(fs.sampleFreq)*1000.0) // change to ms
		success = "LFM waveform with match filtering"
	}
	plot.Location = location

	if len(plot.Status) == 0 {
		plot.Status = success
	}

	// Write to HTTP using template and grid
	if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
		log.Fatalf("Write to HTTP output using template with error: %v\n", err)
	}
}

// handleFilterSignal filters the signal and sends the HTML to the browser for display
func handleFilterSignal(w http.ResponseWriter, r *http.Request) {

	var plot PlotT

	// need number of samples and sample frequency to continue
	temp := r.FormValue("samples")
	if len(temp) > 0 {
		samples, err := strconv.Atoi(temp)
		if err != nil {
			plot.Status = fmt.Sprintf("Samples conversion to int error: %v", err.Error())
			fmt.Printf("Samples conversion to int error: %v\n", err.Error())
			// Write to HTTP using template and grid
			if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
				log.Fatalf("Write to HTTP output using template with error: %v\n", err)
			}
			return
		}

		temp = r.FormValue("samplefreq")
		sf, err := strconv.Atoi(temp)
		if err != nil {
			fmt.Printf("Sample frequency conversion error: %v\n", err)
			plot.Status = fmt.Sprintf("Samples frequency conversion to int error: %v", err.Error())
			// Write to HTTP using template and grid
			if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
				log.Fatalf("Write to HTTP output using template with error: %v\n", err)
			}
			return
		}

		filterState := FilterState{firstSampleTime: 0.0, lastSampleTime: 0.0,
			lastPulseIndex: 0, lastFiltered: make([]float64, 0)}

		// create FilterSignal instance fs
		fs := FilterSignal{
			sema1:            make(chan int),
			sema2:            make(chan int),
			buf:              make([][]float64, 2),
			done:             make(chan struct{}),
			samples:          samples,
			samplesGenerated: 0,
			samplesFiltered:  0,
			sampleFreq:       sf,
			FilterState:      filterState,
			filterCoeff:      make([]float64, 0),
			Endpoints:        Endpoints{nmax: 0, ymin: 0, ymax: 0, xmin: 0, xmax: 0},
		}
		fs.buf[0] = make([]float64, block)
		fs.buf[1] = make([]float64, block)

		// start generating samples and send to filter
		err = fs.generate(r)
		if err != nil {
			plot.Status = err.Error()
			fmt.Printf("generate error: %v\n", err.Error())
			// Write to HTTP using template and grid
			if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
				log.Fatalf("Write to HTTP output using template with error: %v\n", err)
			}
			return
		}

		// start filtering when the samples arrive from the generator
		err = fs.filter(r, &plot)
		if err != nil {
			plot.Status = err.Error()
			fmt.Printf("filter error: %v\n", err.Error())
			// Write to HTTP using template and grid
			if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
				log.Fatalf("Write to HTTP output using template with error: %v\n", err)
			}
			return
		}

		// wait for the generator and filter goroutines to complete
		fs.wg.Wait()

		if err != nil {
			plot.Status = err.Error()
		}

		// Fill in the PlotT grid with the signal time vs amplitude
		err = fs.gridFillInterp(&plot)
		if err != nil {
			plot.Status = err.Error()
			fmt.Printf("gridFillInterp error: %v\n", err.Error())
			// Write to HTTP using template and grid
			if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
				log.Fatalf("Write to HTTP output using template with error: %v\n", err)
			}
			return
		}

		//  generate x-labels, ylabels, status in PlotT and execute the data on the HTML template
		fs.labelExec(w, &plot)

	} else {
		plot.Status = "Enter samples, sample frequency, SNR, where, pulsewidth, bandwidth"
		if err := hideAndSeekTmpl.Execute(w, plot); err != nil {
			log.Fatalf("Write to HTTP output using template with error: %v\n", err)
		}
	}
}

// executive program
func main() {
	// Setup http server with handler for generating and filtering noisy signals
	http.HandleFunc(patternhideandseek, handleFilterSignal)

	fmt.Printf("Hide and Seek Server listening on %v.\n", addr)

	http.ListenAndServe(addr, nil)
}
