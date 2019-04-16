function [ SNR_decoded_dB11, SNR_decoded_dB12, SNR_decoded_dB21, SNR_decoded_dB22, rx_signal ] = decode()
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % wl_example_siso_ofdm_txrx.m
    % A detailed write-up of this example is available on the wiki:
    % http://warpproject.org/trac/wiki/WARPLab/Examples/OFDM
    %
    % Copyright (c) 2015 Mango Communications - All Rights Reserved
    % Distributed under the WARP License (http://warpproject.org/license)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Rx Site
    % Params:
    % USE_WARPLAB_TXRX: Enable WARPLab-in-the-loop (otherwise sim-only)
    % WRITE_PNG_FILES: Enable writing plots to PNG
    % CHANNEL: Channel to tune Tx and Rx radios
    USE_WARPLAB_TXRX        = 0;
    WRITE_PNG_FILES         = 0;
    CHANNEL                 = 11;

    % Waveform params
    % N_OFDM_SYMS: Number of OFDM symbols
    % MOD_ORDER: Modulation order (2/4/16/64 = BSPK/QPSK/16-QAM/64-QAM)
    % TX_SCALE: Scale for Tx waveform ([0:1]) (TX gain)
    N_OFDM_SYMS             = 50;         
    MOD_ORDER               = 2;         
    TX_SCALE                = 1.0;

    % OFDM params
    % SC_IND_PILOTS: Pilot subcarrier indices, index 1 is for DC and index 28 to 28 is not usesd form leakage.
    % SC_IND_DATA: Data subcarrier indices
    % N_SC: Number of subcarriers
    % CP_LEN: Cyclic prefix length
    % N_DATA_SYMS: Number of data symbols (one per data-bearing subcarrier per OFDM symbol)
    % INTERP_RATE: Interpolation rate (must be 2)
    SC_IND_PILOTS           = [ 8, 22, 44, 58 ];                                             
    SC_IND_DATA             = [ 2 : 7, 9 : 21, 23 : 27, 39 : 43, 45 : 57, 59 : 64 ];       
    N_SC                    = 64;                                                               
    CP_LEN                  = 16;                                                               
    N_DATA_SYMS             = N_OFDM_SYMS * length(SC_IND_DATA);                                
    INTERP_RATE             = 2;                                                                

    % Rx processing params
    % FFT_OFFSET: Number of CP samples to use in FFT (on average)
    FFT_OFFSET                    = 4;          
    % Normalized threshold for LTS correlation for packet detection
    % DO_APPLY_CFO_CORRECTION: Enable CFO estimation/correction
    % DO_APPLY_PHASE_ERR_CORRECTION: Enable Residual CFO estimation/correction
    % DO_APPLY_SFO_CORRECTION: Enable SFO estimation/correction
    LTS_CORR_THRESH               = 0.8;         
    DO_APPLY_CFO_CORRECTION       = 0;           
    DO_APPLY_PHASE_ERR_CORRECTION = 0;           
    DO_APPLY_SFO_CORRECTION       = 0;           
    DECIMATE_RATE                 = INTERP_RATE;

    % WARPLab experiment params
    % MAX_TX_LEN: Maximum number of samples to use for this experiment
    MAX_TX_LEN              = 2 ^ 20;      

    %% David: No need in this lab
    %{
    if USE_WARPLAB_TXRX
        %% Set up the WARPLab experiment
        NUMNODES = 2;

        % Create a vector of node objects
        nodes   = wl_initNodes( NUMNODES );
        node_tx = nodes( 1 );
        node_rx = nodes( 2 );

        % Create a UDP broadcast trigger and tell each node to be ready for it
        eth_trig = wl_trigger_eth_udp_broadcast;
        wl_triggerManagerCmd( nodes, 'add_ethernet_trigger', [ eth_trig ] );

        % Read Trigger IDs into workspace
        trig_in_ids  = wl_getTriggerInputIDs( nodes( 1 ) );
        trig_out_ids = wl_getTriggerOutputIDs( nodes( 1 ) );

        % For both nodes, we will allow Ethernet to trigger the buffer baseband and the AGC
        wl_triggerManagerCmd( nodes, 'output_config_input_selection', [ trig_out_ids.BASEBAND, trig_out_ids.AGC ], [ trig_in_ids.ETH_A ] );

        % Set the trigger output delays.
        nodes.wl_triggerManagerCmd( 'output_config_delay', [ trig_out_ids.BASEBAND ], 0 );
        nodes.wl_triggerManagerCmd( 'output_config_delay', [ trig_out_ids.AGC ], TRIGGER_OFFSET_TOL_NS );

        % Get IDs for the interfaces on the boards. 
        ifc_ids_TX = wl_getInterfaceIDs( node_tx );
        ifc_ids_RX = wl_getInterfaceIDs( node_rx );

        % Set up the TX / RX nodes and RF interfaces
        TX_RF     = ifc_ids_TX.RF_A;
        TX_RF_VEC = ifc_ids_TX.RF_A;
        TX_RF_ALL = ifc_ids_TX.RF_ALL;

        RX_RF     = ifc_ids_RX.RF_A;
        RX_RF_VEC = ifc_ids_RX.RF_A;
        RX_RF_ALL = ifc_ids_RX.RF_ALL;

        % Set up the interface for the experiment
        wl_interfaceCmd( node_tx, TX_RF_ALL, 'channel', 2.4, CHANNEL );
        wl_interfaceCmd( node_rx, RX_RF_ALL, 'channel', 2.4, CHANNEL );

        wl_interfaceCmd( node_tx, TX_RF_ALL, 'tx_gains', 3, 30 );

        if USE_AGC
            wl_interfaceCmd( node_rx, RX_RF_ALL, 'rx_gain_mode', 'automatic' );
            wl_basebandCmd( nodes, 'agc_target', -13 );
        else
            wl_interfaceCmd( node_rx, RX_RF_ALL, 'rx_gain_mode', 'manual' );
            % Rx RF Gain in [ 1 : 3 ]
            RxGainRF = 2;
            % Rx Baseband Gain in [ 0 : 31 ]
            RxGainBB = 12;                 
            wl_interfaceCmd( node_rx, RX_RF_ALL, 'rx_gains', RxGainRF, RxGainBB );
        end

        % Get parameters from the node
        SAMP_FREQ = wl_basebandCmd( nodes( 1 ), 'tx_buff_clk_freq' );
        Ts  = 1 / SAMP_FREQ;

        % We will read the transmitter's maximum I/Q buffer length and assign that value to a temporary variable.
        % NOTE:  We assume that the buffers sizes are the same for all interfaces
        maximum_buffer_len = min( MAX_TX_LEN, wl_basebandCmd( node_tx, TX_RF_VEC, 'tx_buff_max_num_samples' ) );
        example_mode_string = 'hw';
    else
        % Use same defaults for hardware-dependent params in sim-only version
        maximum_buffer_len  = min( MAX_TX_LEN, 2 ^ 20 );
        SAMP_FREQ           = 40e6;
        example_mode_string = 'sim';
    end
    %}

    
    %% David: Load files
    file = fopen( '../out/tx_freq_vec.bin' );
    same_real = fread( file, [ 48, 50 ], 'float' );
    same_imag = fread( file, [ 48, 50 ], 'float' );
    fclose( file );
    tx_syms_mat = complex( same_real, same_imag );

    rx_vec_air = read_complex_binary( '../out/rx_vec_air.bin' );
    rx_signal = abs( rx_vec_air );
    % end
    
    
    % LTS for CFO and channel estimation
    lts_f = [ 0, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1 ];

    % Define the pilot tone values as BPSK symbols
    pilots = [ 1, 1, -1, 1 ].';

    % Repeat the pilots across all OFDM symbols
    pilots_mat = repmat( pilots, 1, N_OFDM_SYMS );


    %% David: No need in this lab
    %{
    %% Decimate
    if INTERP_RATE ~= 2
       fprintf( 'Error: INTERP_RATE must equal 2\n' ); 
       return;
    end
    %}

    %% David: No need in this lab
    %{
    raw_rx_dec = filter( interp_filt2, 1, rx_vec_air );
    raw_rx_dec = raw_rx_dec( 1 : 2 : end );
    %}
    raw_rx_dec = rx_vec_air;


    %% David: No need in this lab
    %{
    %% Correlate for LTS

    % Complex cross correlation of Rx waveform with time-domain LTS
    lts_corr = abs( conv( conj( fliplr( lts_t ) ), sign( raw_rx_dec ) ) );

    % Skip early and late samples - avoids occasional false positives from pre-AGC samples
    lts_corr = lts_corr( 32 : end - 32 );

    % Find all correlation peaks
    lts_peaks = find( lts_corr( 1 : 800 ) > LTS_CORR_THRESH * max( lts_corr ) );

    % Select best candidate correlation peak as LTS-payload boundary
    [ LTS1, LTS2 ] = meshgrid( lts_peaks, lts_peaks );
    [ lts_second_peak_index, y ] = find( LTS2 - LTS1 == length( lts_t ) );

    % Stop if no valid correlation peak was found
    if isempty( lts_second_peak_index )
        fprintf( 'No LTS Correlation Peaks Found!\n' );
        return;
    end
    %}
    % end

    %% David: Set the sample indices of the payload symbols and preamble
    % The "+32" corresponds to the 32-sample cyclic prefix on the preamble LTS
    % The "-160" corresponds to the length of the preamble LTS (2.5 copies of 64-sample LTS)
    payload_ind = 640 * 2 + 1;
    lts_ind1 = payload_ind - 640 - 160;
    lts_ind2 = lts_ind1 + 640;
    % end

    if DO_APPLY_CFO_CORRECTION
        %% David: Extract LTS for each Tx1 and Tx2 (not yet CFO corrected)
        rx_lts1 = raw_rx_dec( lts_ind1 : lts_ind1 + 159 );
        rx_lts11 = rx_lts1( -64 + -FFT_OFFSET + [ 97 : 160 ] );
        rx_lts12 = rx_lts1( -FFT_OFFSET + [ 97 : 160 ] );
        % end

        %% David: Calculate coarse CFO est
        rx_cfo_est_lts1 = mean( unwrap( angle( rx_lts2 .* conj( rx_lts1 ) ) ) );
        rx_cfo_est_lts1 = rx_cfo_est_lts1 / ( 2 * pi * 64 );
        % end
    else
        rx_cfo_est_lts1 = 0;
    end

    %% David: Apply CFO correction to raw Rx waveform
    rx_cfo_corr_t1 = exp( -1i * 2 * pi * rx_cfo_est_lts1 * [ 0 : length( raw_rx_dec ) - 1 ] );
    rx_dec_cfo_corr1 = raw_rx_dec .* rx_cfo_corr_t1;
    % end

    
    %% David: Re-extract LTS for channel estimate
    rx_lts1 = rx_dec_cfo_corr1( lts_ind1 : lts_ind1 + 159 );
    rx_lts11 = rx_lts1( -64 + -FFT_OFFSET + [ 97 : 160 ] );
    rx_lts12 = rx_lts1( -FFT_OFFSET + [ 97 : 160 ] );
    
    rx_lts1_f1 = fft( rx_lts11 );
    rx_lts1_f2 = fft( rx_lts12 );
    
    rx_lts2 = rx_dec_cfo_corr2( lts_ind2 : lts_ind2 + 159 );
    rx_lts21 = rx_lts2( -64 + -FFT_OFFSET + [ 97 : 160 ] );
    rx_lts22 = rx_lts2( -FFT_OFFSET + [ 97 : 160 ] );
    
    rx_lts2_f1 = fft( rx_lts21 );
    rx_lts2_f2 = fft( rx_lts22 );
    % end

    
    %% David: Calculate channel estimate from average of 2 training symbols
    rx_H1_est = lts_f .* ( rx_lts1_f1 + rx_lts1_f2 ) / 2;
    rx_H2_est = lts_f .* ( rx_lts2_f1 + rx_lts2_f2 ) / 2;
    % end

    
    %% Rx payload processing
    % Extract the payload samples (integral number of OFDM symbols following preamble)
    payload_vec = rx_dec_cfo_corr1( payload_ind : payload_ind + 4 * N_OFDM_SYMS * ( N_SC + CP_LEN ) - 1 );
    payload_mat = reshape( payload_vec, ( N_SC + CP_LEN ), 4 * N_OFDM_SYMS );
    
    
    %% David: Remove the cyclic prefix, keeping FFT_OFFSET samples of CP (on average)
    payload_mat_noCP = payload_mat( CP_LEN - FFT_OFFSET + [ 1 : N_SC ], : );
    % end
    
    
    %% David: Take the FFT
    syms_f_mat = fft( payload_mat_noCP, N_SC, 1 );
    % end
    
    
    %% David: Equalize (zero-forcing, just divide by complex channel estimates)
    syms_eq_mat11 = syms_f_mat( :, 1 : N_OFDM_SYMS ) ./ repmat( rx_H1_est.', 1, N_OFDM_SYMS );
    % end

    
    if DO_APPLY_SFO_CORRECTION
        % SFO manifests as a frequency-dependent phase whose slope increases
        % over time as the Tx and Rx sample streams drift apart from one
        % another. To correct for this effect, we calculate this phase slope at
        % each OFDM symbol using the pilot tones and use this slope to
        % interpolate a phase correction for each data-bearing subcarrier.

        
        %% David: Extract the pilot tones and "equalize" them by their nominal Tx values
        pilots_f_mat11 = syms_eq_mat11( SC_IND_PILOTS , : );
        pilots_f_mat_comp11 = pilots_f_mat11 .* pilots_mat;
        % end

        
        %% David: Calculate the phases of every Rx pilot tone
        pilot_phases11 = unwrap( angle( fftshift( pilots_f_mat_comp11, 1 ) ), [], 1 );
        % end

        
        %% David: Calculate slope of pilot tone phases vs frequency in each OFDM symbol
        pilot_spacing_mat = repmat( mod( diff( fftshift( SC_IND_PILOTS ) ), 64 ).', 1, N_OFDM_SYMS ); 
        pilot_slope_mat = mean( diff( pilot_phases11 ) ./ pilot_spacing_mat );
        % end

        
        %% David: Calculate the SFO correction phases for each OFDM symbol
        pilot_phase_sfo_corr = fftshift( ( -32 : 31 ).' * pilot_slope_mat, 1 );
        pilot_phase_corr = exp( -1i * ( pilot_phase_sfo_corr ) );
        % end

        
        %% David: Apply the pilot phase correction per symbol
        syms_eq_mat11 = syms_eq_mat11 .* pilot_phase_corr;
        % end
    else
        % Define an empty SFO correction matrix (used by plotting code below)
        pilot_phase_sfo_corr = zeros( N_SC, N_OFDM_SYMS );
    end


    if DO_APPLY_PHASE_ERR_CORRECTION
        %% David: Extract the pilots and calculate per-symbol phase error
        pilots_f_mat = syms_eq_mat11( SC_IND_PILOTS, : );
        pilots_f_mat_comp = pilots_f_mat .* pilots_mat;
        pilot_phase_err = angle( mean( pilots_f_mat_comp ) );
        % end
    else
        % Define an empty phase correction vector (used by plotting code below)
        pilot_phase_err = zeros( 1, N_OFDM_SYMS );
    end
    pilot_phase_err_corr = repmat( pilot_phase_err, N_SC, 1 );
    pilot_phase_corr = exp( -1i * ( pilot_phase_err_corr ) );

    
    %% David: Apply the pilot phase correction per symbol
    syms_eq_pc_mat11 = syms_eq_mat11 .* pilot_phase_corr;
    payload_syms_mat11 = syms_eq_pc_mat11( SC_IND_DATA, : );
    % end


    %% David: Demodulate
    rx_syms11 = reshape( payload_syms_mat11, 1, N_DATA_SYMS );
    % end


    demod_fcn_bpsk = @( x ) double( real( x ) > 0 );
    demod_fcn_qpsk = @( x ) double( 2 * ( real( x ) > 0 ) + 1 * ( imag( x ) > 0 ) );
    demod_fcn_16qam = @( x ) ( 8 * ( real( x ) > 0 ) ) + ( 4 * ( abs( real( x ) ) < 0.6325 ) ) + ( 2 * ( imag( x ) > 0 ) ) + ( 1 *( abs( imag( x ) ) < 0.6325 ) );
    demod_fcn_64qam = @( x ) ( 32 * ( real( x ) > 0 ) ) + ( 16 * ( abs( real( x ) ) < 0.6172 ) ) + ( 8 * ( ( abs( real( x ) ) < ( 0.9258 ) ) && ( ( abs( real( x ) ) > ( 0.3086 ) ) ) ) ) + ( 4 * ( imag( x ) > 0 ) ) + ( 2 * ( abs( imag( x ) ) < 0.6172 ) ) + ( 1 * ( ( abs( imag( x ) ) < ( 0.9258 ) ) && ( ( abs( imag( x ) ) > ( 0.3086 ) ) ) ) );


    %% David: Calculate decoded SNR
    N0 = mean( abs( payload_syms_mat11 - tx_syms_mat ) .^ 2 );
    SNR_decoded = mean( abs( tx_syms_mat ) .^ 2 ) / N0;
    SNR_decoded_dB11 = 10 * log10( SNR_decoded );
    % end

    
    switch( MOD_ORDER )
        % BPSK
        case 2         
            rx_data = arrayfun( demod_fcn_bpsk, rx_syms11 );
        % QPSK
        case 4         
            rx_data = arrayfun( demod_fcn_qpsk, rx_syms11 );
        % 16-QAM
        case 16        
            rx_data = arrayfun( demod_fcn_16qam, rx_syms11 );
        % 64-QAM
        case 64        
            rx_data = arrayfun( demod_fcn_64qam, rx_syms11 );
    end

    %% David: Plot Results (no need in this lab)
    cf = 0;

    % Rx signal
    cf = cf + 1;
    figure( cf ); 
    clf;
    plot( rx_signal, 'b' );
    title( 'Rx Signal' );

    if WRITE_PNG_FILES
        print( gcf, sprintf( 'wl_ofdm_plots_%s_rxIQ', example_mode_string ), '-dpng', '-r96', '-painters' );
    end

    % Channel Estimates
    cf = cf + 1;

    rx_H_est_plot = repmat( complex( NaN, NaN ), 1, length( rx_H_est ) );
    rx_H_est_plot( SC_IND_DATA ) = rx_H_est( SC_IND_DATA );
    rx_H_est_plot( SC_IND_PILOTS ) = rx_H_est( SC_IND_PILOTS );

    x = ( 20 / N_SC ) * ( -( N_SC / 2 ) : ( N_SC / 2 - 1 ) );

    figure( cf ); 
    clf;
    subplot( 2, 1, 1 );
    stairs( x - ( 20 / ( 2 * N_SC ) ), fftshift( real( rx_H_est_plot ) ), 'b', 'LineWidth', 2 );
    hold on;
    stairs( x - ( 20 / ( 2 * N_SC ) ), fftshift( imag( rx_H_est_plot ) ), 'r', 'LineWidth', 2 );
    hold off;
    axis( [ min( x ), max( x ), -1.1 * max( abs( rx_H_est_plot ) ), 1.1 * max( abs( rx_H_est_plot ) ) ] );
    grid on;
    title( 'Channel Estimates (I and Q)' );

    subplot( 2, 1, 2 );
    bh = bar( x, fftshift( abs( rx_H_est_plot ) ), 1, 'LineWidth', 1 );
    shading flat;
    set( bh, 'FaceColor', [ 0, 0, 1 ] );
    axis( [ min( x ), max( x ), 0, 1.1 * max( abs( rx_H_est_plot ) ) ] );
    grid on;
    title( 'Channel Estimates (Magnitude)' );
    xlabel( 'Baseband Frequency (MHz)' );

    if WRITE_PNG_FILES
        print( gcf, sprintf( 'wl_ofdm_plots_%s_chanEst', example_mode_string ), '-dpng', '-r96', '-painters' );
    end
    % end
end