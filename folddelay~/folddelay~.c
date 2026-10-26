#include "m_pd.h"
#include <stdlib.h>
#include <math.h>

#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

#define PI 4.0f * atanf(1.0f)

/* ------------------------ folddelay~ ----------------------------- */

/* tilde object to do a simple delay line - showing parameter dezippering
	and a simple clipping algorithm. */

static t_class *folddelay_class;

typedef struct _folddelay
{
    t_object x_obj; 	/* obligatory header */
    t_float delay_time;    	/* floats on the first inlet set the delay time */
	t_float	delay_samples;	/* derived from delay_time and sample rate */
	t_float	delay_samples_current;
	t_float	delay_feedback;	/* second inlet from the right */
	t_float	delay_feedback_current;
	t_float sample_rate;
	t_int recordpostion;	/* the actual integer position in the delay 
								table for writing new samples */
	t_int	delayline_size;	/* number of samples max for delay line */
	t_float *delayline;	/* a place to hold the samples */ 

    t_float attackF;
    t_float releaseF;
    t_float peak;
} t_folddelay;


static t_float getPeakAttackRelease(t_folddelay *x, float sample) {
    float attackMult;
    float releaseMult;
    
    // rectify sample
    if(sample < 0.0) { sample = -sample; }
    
    // filter
    attackMult = exp((-2 * PI * x->attackF)/x->sample_rate);
    releaseMult = exp((-2 * PI * x->releaseF)/x->sample_rate);

    if(sample > x->peak) {
        x->peak = attackMult * (x->peak - sample) + sample;
    } else {
        x->peak = releaseMult * (x->peak - sample) + sample;
    }

    return x->peak;
}



static t_int *folddelay_perform(t_int *w)
{
	t_folddelay *x = (t_folddelay *)(w[1]);
    t_float *in = (t_float *)(w[2]);
    t_float *delay = (t_float *)(w[3]);
    t_float *out = (t_float *)(w[4]);
    int n = (int)(w[5]);

	// i like counting from zero, so i use sample to count the offset from
	// the start of the in and out blocks
	int blocksize = n;
	int i, sample = 0;
	float playposfloat, playposfrac;
    long playpos, playposp1;
    float insample, outsample, delayinput;

	float delay_increment, feedback_increment;
	float oneoverblocksize = 1.0f/(float)blocksize;
    
    x->delay_samples = x->delay_time * x->sample_rate;

	// calculate the delay time in samples
	delay_increment = (x->delay_samples - x->delay_samples_current) * oneoverblocksize;

    while (n--)
    {
	//	x->delay_samples = *(delay+sample) * x->sample_rate;

        // get current sample
        insample = *(in+sample);
        
        // determine desired feedback (& increment to reach it) based on peak determined by envelope follower
        x->delay_feedback = 1.5 * getPeakAttackRelease(x, insample);
        feedback_increment = (x->delay_feedback - x->delay_feedback_current) * oneoverblocksize;

		// update the delay_samples and feedback for smooth changes
		x->delay_samples_current += delay_increment;
		x->delay_feedback_current += feedback_increment;
        

		// calculate the new position in the delayline to take the output
		playposfloat = (float)(x->recordpostion) + x->delay_samples_current;

        
		// make sure we don't run over the ends of the table
        
        playpos = (long)playposfloat;
        playposfrac = playposfloat - (float)playpos;
        playposp1 = playpos + 1;

		while(playpos < 0)
			playpos += x->delayline_size;
		while(playpos >= x->delayline_size)
			playpos -= x->delayline_size;
        
		while(playposp1 < 0)
			playposp1 += x->delayline_size;
		while(playposp1 >= x->delayline_size)
			playposp1 -= x->delayline_size;
        
		// look into the delayline and grab the output
		 outsample = *(x->delayline+playpos)* (1.f - playposfrac)
                        + *(x->delayline+playposp1)*playposfrac;

		// multiply the output by feedback, add it to the input, and pop 
		// it back in the delayline
		delayinput = outsample * x->delay_feedback_current + insample;
        
		/*
        if(delayinput > 1.0f)
            delayinput = 1.0f;
        if(delayinput < -1.0f)
            delayinput = -1.0f;
        */
        delayinput = 1.0f * delayinput - (delayinput * delayinput * delayinput * .5f);
        
		*(x->delayline+x->recordpostion) = delayinput;
        
        float sampsout = outsample+insample;
        if(sampsout >= 1) {
            printf("\ngain = %f", sampsout);
        }
		//*(out+sample) = outsample+insample; //ORIGINAL
		// use wavefolding to keep the output signal under 1.0
        *(out+sample) = cos(0.5 * PI * sampsout)
        - 1.0/9.0 * cos(1.5 * PI * sampsout)
        + 1.0/25.0 * cos(2.5 * PI * sampsout)
        - 1.0/49.0 * cos(3.5 * PI * sampsout);

        
		sample++;
		x->recordpostion--;
		if(x->recordpostion < 0)
			x->recordpostion = x->delayline_size - 1;
    }
    return (w+6);
}

    /* called to start DSP.  Here we call Pd back to add our perform
    routine to a linear callback list which Pd in turn calls to grind
    out the samples. */
static void folddelay_dsp(t_folddelay *x, t_signal **sp)
{
	x->sample_rate = sp[0]->s_sr;
    dsp_add(folddelay_perform, 5, x, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[0]->s_n);
}

/*
void folddelay_feedback(t_folddelay *x, t_floatarg g)
{
    x->delay_feedback = g;
}
*/

static void *folddelay_new(void)
{
	 long i;
    t_folddelay *x = (t_folddelay *)pd_new(folddelay_class);
	inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("signal"), 0);
    //inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("feedback"));
    floatinlet_new(&x->x_obj, &x->attackF);
    floatinlet_new(&x->x_obj, &x->releaseF);
    outlet_new(&x->x_obj, gensym("signal"));
    x->delay_time = 1.5f;
    x->delay_samples = 0.0f;
	x->delay_feedback = 0.5f;
	x->recordpostion = 0;
	x->delayline_size = 16777216; // 64MB- maybe a little big?

	x->delayline = (t_float *)malloc(x->delayline_size * sizeof(t_float));
	for(i=0; i<x->delayline_size;i++)
		*(x->delayline+i) = 0.0f;
    return (x);
}

// since we allocated some memory, we need a delete function
static void folddelay_free(t_folddelay *x)
{
	if(x->delayline != 0) {
        free(x->delayline);
        x->delayline = 0;
    }
}


    /* this routine, which must have exactly this name (with the "~" replaced
    by "_tilde) is called when the code is first loaded, and tells Pd how
    to build the "class". */
void folddelay_tilde_setup(void)
{
    folddelay_class = class_new(gensym("folddelay~"), (t_newmethod)folddelay_new, (t_method)folddelay_free,
    	sizeof(t_folddelay), 0, A_DEFFLOAT, 0);
	    /* this is magic to declare that the leftmost, "main" inlet
	    takes signals; other signal inlets are done differently... */
		/* also installs delay_time as the leftmost inlet float */
    CLASS_MAINSIGNALIN(folddelay_class, t_folddelay, delay_time);
    	/* here we tell Pd about the "dsp" method, which is called back
	when DSP is turned on. */
    class_addmethod(folddelay_class, (t_method)folddelay_dsp, gensym("dsp"), (t_atomtype)0);
    //class_addmethod(folddelay_class, (t_method)folddelay_feedback, gensym("feedback"), A_FLOAT, 0);
}