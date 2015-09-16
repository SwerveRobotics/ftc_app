package org.swerverobotics.library.internal;

import java.util.concurrent.Semaphore;

/**
 * A class that helps us start a thread and interlock with its actual starting up.
 *
 * It's surprisingly often that in order to be able to correctly shut down a thread after
 * it has begun, or to give the thread the opportunity to acquire resources it needs to operate,
 * that one shouldn't return from the logic that 'starts' the thread until the thread has
 * begun execution and positively indicated that it's good to go.
 *
 * This class helps to implement that handshake logic.
 */
public class HandshakeThreadStarter
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------
    
    public String  getName()            { return this.name;   }
    public void    setName(String name) { this.name = name;   }
    public Thread  getThread()          { return this.thread; }

    private         String          name;
    private         IHandshakeable  shakeable;
    private final   Semaphore       semaphore      = new Semaphore(0);   // no permits initially; it is 'reset';
    private         Thread          thread         = null;
    private         boolean         started        = false;
    private         boolean         stopRequested  = false;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public HandshakeThreadStarter(String name, IHandshakeable shakeable)
        {
        this.shakeable = shakeable;
        this.name      = name;
        }
    public HandshakeThreadStarter(IHandshakeable shakeable)
        {
        this(null, shakeable);
        }

    //----------------------------------------------------------------------------------------------
    // Public methods
    //----------------------------------------------------------------------------------------------

    /** Get everything ready for starting */
    public synchronized Thread prepareStart()
        {
        stop();

        resetEvent();
        this.stopRequested = false;
        this.thread = new Thread(new Runner());
        if (this.name != null)
            this.thread.setName(this.name);

        return this.thread;
        }

    /** Starts the thread going. Blocks until the thread actually runs and calls starter.handshake(). */
    public synchronized void start() throws InterruptedException
        {
        try {
            if (this.thread == null)
                prepareStart();

            this.thread.start();
            waitEvent();

            this.started = true;
            }
        catch (InterruptedException ex)
            {
            // Clean up if we were interrupted while waiting
            this.started = true;    // so stop() will do the work
            stop();
            throw ex;               // pass it on
            }
        }

    /** Returns whether the thread is, for the moment, started */
    public synchronized boolean isStarted()
        {
        return this.started;
        }

    /** Requests that the thread stop at its earliest available opportunity */
    public synchronized void requestStop()
        {
        this.stopRequested = true;
        if (this.thread != null)
            this.thread.interrupt();
        }

    /** Stops the thread, if currently running. Blocks until thread terminates. */
    public synchronized void stop()
        {
        if (this.started)
            {
            try {
                this.stopRequested = true;
                this.thread.interrupt();
                this.thread.join();
                }
            catch (InterruptedException ignored)
                {
                }
            finally
                {
                this.thread  = null;
                this.started = false;
                }
            }
        }

    /** block until the thread terminates. Note that this does not itself stop the thread */
    public void join() throws InterruptedException
        {
        Thread thread;
        synchronized (this)
            {
            thread = this.thread;
            }
        if (thread != null)
            thread.join();
        }

    public void join(int ms) throws InterruptedException
        {
        Thread thread;
        synchronized (this)
            {
            thread = this.thread;
            }
        if (thread != null)
            thread.join(ms);
        }
    //----------------------------------------------------------------------------------------------
    // For use by thread
    //----------------------------------------------------------------------------------------------

    /** called by the thread to indicate that he's alive and well */
    public void handshake()
        {
        this.setEvent();
        }

    /** Returns whether the thread has been asked to stop */
    public synchronized boolean stopRequested()
        {
        return this.stopRequested || (this.thread != null && this.thread.isInterrupted());
        }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    class Runner implements Runnable
        {
        @Override public void run()
            {
            shakeable.run(HandshakeThreadStarter.this);
            }
        }

    // THIS HAS DEADLOCK

    /* make it so that subsequent waiters will block */
    void resetEvent()
        {
        synchronized (this.semaphore)
            {
            // Make the semaphore have zero permits. Thus, subsequent acquirers will have to wait.
            this.semaphore.drainPermits();
            }
        }

    /* wait until setEvent() has happened. leave state unchanged as a result of our waiting */
    void waitEvent() throws InterruptedException
        {
        synchronized (this.semaphore)
            {
            this.semaphore.acquire();       // get a permit
            this.semaphore.release();       // give it back
            }
        }

    /* make it so that subsequent waiters will not block */
    void setEvent()
        {
        synchronized (this.semaphore)
            {
            // Make the semaphore have one permit
            this.semaphore.drainPermits();
            this.semaphore.release();
            }
        }
    }

